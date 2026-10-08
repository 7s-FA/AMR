#!/usr/bin/env python3
"""Bounded read-only AMR profiling. Never publishes goals, GPIO or motor commands."""
import argparse,json,os,signal,time,statistics,math,subprocess
from collections import deque
from pathlib import Path

def cpu():
    v=list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:]))
    return sum(v),v[3]+v[4]

_commands={}
def processes():
    result={}
    for p in Path('/proc').glob('[0-9]*'):
        try:
            text=(p/'stat').read_text();f=text[text.rfind(')')+2:].split()
            cached=_commands.get(p.name)
            if cached is None or cached[0]!=f[19]:
                command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
                _commands[p.name]=(f[19],command)
            else:command=cached[1]
            if command:result[p.name]=(int(f[11])+int(f[12]),int(f[21])*os.sysconf('SC_PAGE_SIZE'),command[:240])
        except (OSError,ValueError,IndexError):pass
    return result

def udp():
    rows=Path('/proc/net/snmp').read_text().splitlines()
    for i,row in enumerate(rows):
        if row.startswith('Udp:'):return dict(zip(row.split()[1:],map(int,rows[i+1].split()[1:])))
    return {}

def percentiles(values):
    if not values:return {}
    v=sorted(values)
    return {'p50':round(statistics.median(v),3),'p95':round(v[int(.95*(len(v)-1))],3),'max':round(max(v),3)}

def main():
    a=argparse.ArgumentParser();a.add_argument('robot',choices=['burger1','burger2'])
    a.add_argument('--duration',type=float,default=600);a.add_argument('--interval',type=float,default=1.)
    a.add_argument('--output',required=True)
    args=a.parse_args()
    if args.interval<.25 or not 0<args.duration<=1800:raise ValueError('Bounded profiling only')
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    binary=Path(__file__).with_name('performance_topics')
    if not binary.exists():raise RuntimeError('Build/install performance_topics before profiling')
    topic_path=path.with_suffix('.topics.json')
    observer=subprocess.Popen([str(binary),args.robot,str(topic_path)])
    root=Path(__file__).absolute().parents[3]
    stopping=False
    def halt(*_):
        nonlocal stopping;stopping=True
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt)
    start=time.monotonic();deadline=start+args.duration;due=start+args.interval
    old_cpu=cpu();old_process=processes();old_udp=udp();old_time=start
    peaks={};samples=0;counts={};summary=dict(robot=args.robot,interval_s=args.interval,
        pid=os.getpid(),measurement='observed peaks; callback CPU excludes DDS/deserialization')
    try:
        with path.open('w',buffering=1) as out:
            while not stopping and time.monotonic()<deadline:
                time.sleep(.05)
                if observer.poll() is not None:raise RuntimeError("Native observer exited")
                now=time.monotonic()
                if now<due:continue
                due=now+args.interval;elapsed=now-old_time;new_cpu=cpu();new_process=processes();new_udp=udp()
                total=100*(1-(new_cpu[1]-old_cpu[1])/max(1,new_cpu[0]-old_cpu[0]))
                top=[]
                for pid,(ticks,rss,command) in new_process.items():
                    if pid not in old_process:continue
                    usage=(ticks-old_process[pid][0])/os.sysconf('SC_CLK_TCK')/elapsed*100/(os.cpu_count() or 4)
                    if usage>.025:top.append(dict(pid=pid,cpu_total_pct=round(usage,3),rss_mb=round(rss/1048576,2),command=command))
                top.sort(key=lambda p:p['cpu_total_pct'],reverse=True)
                try:task=json.loads((root/'data'/args.robot/'mission_state.json').read_text())
                except (OSError,ValueError):task={}
                phase=task.get('stage','unknown') if task.get('status')=='running' else 'idle_'+task.get('status','unknown')
                try:telemetry=json.loads(topic_path.read_text())
                except (OSError,ValueError):telemetry={}
                dock=telemetry.get('dock',{})
                if telemetry.get('dock_age_s',1e9)<2 and dock.get('state') in ('ALIGN','VISION_WAIT','ODOM_WAIT','FINAL_APPROACH','STOPPING'):
                    phase='terminal_'+dock['state']
                peak=peaks.setdefault(phase,dict(samples=0,cpu=[],memory=[]))
                peak['samples']+=1;peak['cpu'].append(total)
                memory={k:int(v.split()[0]) for k,v in (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
                used_mb=(memory['MemTotal']-memory['MemAvailable'])/1024;peak['memory'].append(used_mb)
                topics=telemetry.get('topics',{})
                record=dict(unix=time.time(),elapsed_s=round(now-start,3),phase=phase,
                    cpu_total_pct=round(total,3),observer_cpu_total_pct=round(sum(p['cpu_total_pct'] for p in top if p['pid'] in (str(os.getpid()),str(observer.pid))),3),memory_used_mb=round(used_mb,2),
                    temperature_c=int(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000,
                    processes=top[:18],topics=topics,command_id=task.get('command_id'),destination=task.get('destination'),status=task.get('status'),
                    udp_delta={k:new_udp[k]-old_udp.get(k,0) for k in new_udp},
                    data_processing=dock.get('data_processing'),ir=telemetry.get('ir'),topic_observer_age_s=now-telemetry.get('monotonic',now),cpu_frequency_khz=int(Path('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq').read_text()))
                out.write(json.dumps(record,ensure_ascii=False)+'\n');samples+=1
                old_cpu,old_process,old_udp,old_time=new_cpu,new_process,new_udp,now
    finally:
        summary.update(elapsed_s=round(time.monotonic()-start,3),samples=samples,
            phases={k:dict(samples=v['samples'],cpu_total_pct=percentiles(v['cpu']),memory_used_mb=percentiles(v['memory'])) for k,v in peaks.items()})
        path.with_suffix('.summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        observer.send_signal(signal.SIGINT)
        try:observer.wait(timeout=4)
        except subprocess.TimeoutExpired:observer.kill();observer.wait()

if __name__=='__main__':main()

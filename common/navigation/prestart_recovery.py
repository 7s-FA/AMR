"""Release only a proven first docking attempt that never issued motion.

Caller holds Operation.lock(), has verified fresh stationary odom and no worker.
No pose writes, motor commands, or automatic resumption.
"""
import json
from pathlib import Path
import time


def proof(data):
    data=Path(data)
    try:
        flag=data/'departure_pending'
        if flag.read_text().strip()!='terminal_started_dock':return None
        evidence=json.loads((data/'terminal_result.json').read_text())
        report=evidence['controller_report']
        reason='auto_start_timeout: four_markers_required'
        if (evidence['mode']!='dock' or evidence['status']!='failed'
                or report['state']!='FAULT' or report['reason']!=reason
                or report['docking_encoder']!={'valid':False,'start':None,'current':None}):return None
        end=evidence['completed_unix'];begin=flag.stat().st_mtime
        if not 0<=end-begin<=120:return None
        destination=json.loads((data/'terminal_attempt.json').read_text())['destination']
        if destination not in ('mat','asm'):return None
        matches=[]
        for p in (data/'commands').glob('*.json'):
            r=json.loads(p.read_text())
            if (r.get('destination')==destination and r.get('status')=='failed'
                    and r.get('retry_step')=='terminal_dock' and r.get('retry_attempt')==1
                    and r.get('started_unix',end+1)<=begin<=end<=r.get('updated_unix',0)
                    and 'missing or stale encoder' in r.get('error','')):matches.append(r)
        if len(matches)!=1:return None
        job=matches[0]
        log=Path(evidence['log_file']).resolve()
        if log.parent!=(data/'terminal_logs').resolve():return None
        # Reject later retries: the whole mission must contain only this attempt.
        logs=[p.resolve() for p in (data/'terminal_logs').glob('*.log')
              if job['started_unix']<=p.stat().st_mtime<=end+.1]
        if logs!=[log]:return None
        lines=log.read_text().splitlines()
        paths=[Path(x.split('Docking diagnostics: ',1)[1]).resolve() for x in lines if x.startswith('Docking diagnostics: ')]
        if len(paths)!=1 or paths[0].parent!=(data/'docking_logs').resolve():return None
        folder=paths[0]
        summary=json.loads((folder/'summary.json').read_text())
        if summary['exit_reason']!='FAULT: '+reason:return None
        last=summary['last_status']
        if last.get('log_dropped_records')!=0 or last.get('log_writer_error') is not None:return None
        rows=[json.loads(x) for x in (folder/'events.jsonl').read_text().splitlines()]
        commands=[r for r in rows if r.get('kind')=='command']
        states=[r for r in rows if r.get('kind')=='status']
        if not commands or len(states)<2 or states[0]['state']!='IDLE' or states[-1]['state']!='FAULT':return None
        for r in commands+states:
            if r.get('linear_mps')!=0 or r.get('angular_rps')!=0:return None
        for r in states:
            if (r['state'] not in ('IDLE','FAULT') or r.get('travel_m')!=0
                    or r.get('final_travel_m')!=0 or r.get('move') is not None
                    or r.get('log_dropped_records')!=0 or r.get('log_writer_error') is not None):return None
        if states[-1]['reason']!=reason:return None
        if not begin<=states[0]['unix_s']<=states[-1]['unix_s']<=end:return None
        return {'command_id':job['command_id'],'destination':destination,
                'terminal_result':evidence,'diagnostics':str(folder),'reason':'verified_prestart_no_motion'}
    except (OSError,ValueError,KeyError,TypeError):return None


def release(data):
    data=Path(data);verified=proof(data)
    if verified is None:return False
    archive=data/'prestart_recovery';archive.mkdir(exist_ok=True)
    # Persist evidence before removing the departure prohibition.
    path=archive/(str(time.time_ns())+'.json')
    path.write_text(json.dumps(verified,ensure_ascii=False,indent=2))
    (data/'departure_pending').unlink()
    return True

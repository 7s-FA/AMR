import json,os,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'common/navigation'))
from prestart_recovery import proof,release

@pytest.fixture
def data(tmp_path):
 def write(p,value):
  p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value))
 d=tmp_path;flag=d/'departure_pending';flag.write_text('terminal_started_dock');os.utime(flag,(100,100))
 write(d/'terminal_attempt.json',{'destination':'mat'})
 write(d/'commands'/'job.json',dict(destination='mat',status='failed',retry_step='terminal_dock',retry_attempt=1,started_unix=90,updated_unix=120,error='missing or stale encoder',command_id='job'))
 folder=d/'docking_logs'/'attempt';folder.mkdir(parents=True)
 reason='auto_start_timeout: four_markers_required'
 common=dict(kind='status',linear_mps=0.,angular_rps=0.,travel_m=0.,final_travel_m=0.,move=None,log_dropped_records=0,log_writer_error=None)
 rows=[dict(common,state='IDLE',unix_s=101),dict(kind='command',linear_mps=0,angular_rps=0),dict(common,state='FAULT',reason=reason,unix_s=110)]
 (folder/'events.jsonl').write_text('\n'.join(map(json.dumps,rows)))
 write(folder/'summary.json',{'exit_reason':'FAULT: '+reason,'last_status':rows[-1]})
 log=d/'terminal_logs'/'attempt.log';log.parent.mkdir();log.write_text('Docking diagnostics: '+str(folder));os.utime(log,(111,111))
 write(d/'terminal_result.json',dict(mode='dock',status='failed',completed_unix=112,log_file=str(log),controller_report=dict(state='FAULT',reason=reason,docking_encoder=dict(valid=False,start=None,current=None))))
 return d

def test_verified_release_archives_and_preserves_coordinates(data):
 (data/'localization_seed.json').write_text('unchanged')
 assert release(data);assert not (data/'departure_pending').exists()
 assert (data/'localization_seed.json').read_text()=='unchanged'
 assert len(list((data/'prestart_recovery').glob('*.json')))==1
 assert not release(data)

@pytest.mark.parametrize('change',['motion','active','drop','missing','stale','retry','another_attempt','encoder'])
def test_uncertain_or_started_keeps_lock(data,change):
 f=data/'docking_logs/attempt/events.jsonl';rows=[json.loads(x) for x in f.read_text().splitlines()]
 if change=='motion':rows[1]['linear_mps']=.01
 if change=='active':rows[0]['state']='ALIGN'
 if change=='drop':rows[-1]['log_dropped_records']=1
 f.write_text('\n'.join(map(json.dumps,rows)))
 if change=='missing':f.unlink()
 if change=='stale':os.utime(data/'departure_pending',(200,200))
 if change=='retry':
  p=data/'commands/job.json';r=json.loads(p.read_text());r['retry_attempt']=2;p.write_text(json.dumps(r))
 if change=='another_attempt':
  p=data/'terminal_logs/older.log';p.write_text('previous');os.utime(p,(99,99))
 if change=='encoder':
  p=data/'terminal_result.json';r=json.loads(p.read_text());r['controller_report']['docking_encoder']['start']=[1,2];p.write_text(json.dumps(r))
 assert not release(data);assert (data/'departure_pending').exists()

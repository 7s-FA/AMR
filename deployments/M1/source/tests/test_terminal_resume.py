import json
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'common/navigation'))
from sequence_runner import interrupted_terminal, Sequence


def test_same_destination_resumes_only_terminal(tmp_path):
    (tmp_path/'departure_pending').write_text('terminal_started_park\n')
    (tmp_path/'mission_state.json').write_text(json.dumps({'destination':'park'}))
    assert interrupted_terminal(tmp_path,'park')
    calls=[]
    Sequence(calls.append,calls.append,{'routes':{'park':[]},'terminal':{'park':'park'}}).execute('park',True)
    assert calls==['park']


def test_different_destination_rejected_without_losing_original(tmp_path):
    (tmp_path/'departure_pending').write_text('terminal_started_dock\n')
    (tmp_path/'mission_state.json').write_text(json.dumps({'destination':'asm'}))
    with pytest.raises(RuntimeError,match='다른 목적지'):interrupted_terminal(tmp_path,'mat')
    (tmp_path/'mission_state.json').write_text(json.dumps({'destination':'mat'}))
    assert interrupted_terminal(tmp_path,'asm')


@pytest.mark.parametrize('flag',['successful_park now','confirmed own parking at startup'])
def test_completed_still_uses_normal_departure(tmp_path,flag):
    (tmp_path/'departure_pending').write_text(flag)
    assert not interrupted_terminal(tmp_path,'park')


def test_unknown_interrupted_destination_blocks_route(tmp_path):
    (tmp_path/'departure_pending').write_text('terminal_started_park')
    with pytest.raises(RuntimeError,match='확인 불가'):interrupted_terminal(tmp_path,'park')

"""Admission and fallback checks without creating ROS nodes or motor publishers."""
import importlib.util
import io
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def load(profile, name):
    path = ROOT/profile/'src/navigation'/name
    spec = importlib.util.spec_from_file_location(profile+'_'+path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('issued', [-10, 0, 11, True, float('nan')])
def test_waypoint_worker_rejects_expired_future_invalid_request(issued):
    worker = load('M1', 'waypoint_worker.py')
    with pytest.raises(ValueError):
        worker.validate_request({'op':'run', 'issued':issued,
                                 'argv':['--namespace','burger1']}, 10)


@pytest.mark.parametrize('argv', [['--namespace','burger2'],
    ['--namespace','burger1','--namespace','burger1'],
    ['--namespace','burger1','--namespace=burger2']])
def test_waypoint_worker_rejects_other_or_ambiguous_robot(argv):
    with pytest.raises(ValueError):
        load('M1', 'waypoint_worker.py').validate_request(
            {'op':'run', 'issued':10, 'argv':argv}, 10.1)


@pytest.mark.parametrize('profile,name', [('M1','waypoint_client.py'), ('M2','rest_client.py')])
@pytest.mark.parametrize('response', [b'{"type":"result","code":2}\n', b''])
def test_connected_request_never_returns_cold_fallback_code(monkeypatch, profile, name, response):
    client = load(profile, name)
    class Connection:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def settimeout(self, value): pass
        def connect(self, address): pass
        def sendall(self, data): pass
        def makefile(self, mode): return io.BytesIO(response)
    monkeypatch.setattr(client, 'available', lambda *args: True)
    monkeypatch.setattr(client.socket, 'socket', lambda *args: Connection())
    monkeypatch.setattr(sys, 'argv', ['client', '/unused/entry', '--dry-run'])
    assert client.main() == 1


@pytest.mark.parametrize('profile,name', [('M1','waypoint_client.py'), ('M2','rest_client.py')])
def test_unavailable_worker_permits_cold_fallback(monkeypatch, profile, name):
    client = load(profile, name)
    monkeypatch.setattr(client, 'available', lambda *args: False)
    monkeypatch.setattr(sys, 'argv', ['client', '/unused/entry'])
    assert client.main() == 2


def test_service_lease_cancels_pending_requests_only():
    worker = load('M1', 'waypoint_worker.py')
    class Future:
        def __init__(self, done): self.finished=done; self.cancelled=False
        def done(self): return self.finished
        def cancel(self): self.cancelled=True
    class Client:
        def __init__(self): self.removed=[]
        def remove_pending_request(self, future): self.removed.append(future)
    client=Client();lease=worker.ServiceLease(client)
    completed=Future(True);pending=Future(False);lease.pending=[completed,pending]
    lease.release()
    assert client.removed == [pending] and pending.cancelled
    assert not completed.cancelled and not lease.pending


@pytest.mark.parametrize('profile', ['M1','M2'])
def test_worker_dependency_closure_in_manifest(profile):
    manifest=json.loads((ROOT/profile/'runtime_manifest.json').read_text())
    entries={e['target']:e['source'] for e in manifest['files']}
    robot='burger'+profile[-1]
    needed=['ensure_rest_ready.sh','start_rest_ready.sh','rest_ready_worker.py']
    needed += (['ensure_waypoint_ready.sh','start_waypoint_worker.sh','waypoint_client.py','waypoint_worker.py']
               if profile=='M1' else ['rest_client.py','rest_ready_io.py'])
    for name in needed:
        assert (ROOT/entries['workspace/robot/'+robot+'/navigation/'+name]).is_file()
    assert 'units/'+robot+'-rest-ready.service' in entries
    if profile=='M1': assert 'units/burger1-waypoint-ready.service' in entries

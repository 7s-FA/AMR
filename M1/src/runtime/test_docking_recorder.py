import json
from docking_recorder import DockingRecorder


def test_run_snapshots_and_drains_pending_events(tmp_path):
    source = tmp_path / 'camera.yaml'
    source.write_text('image_height: 360\n')
    recorder = DockingRecorder(tmp_path / 'run', {'control': {'min_angular_rps': .02}}, [source])
    recorder.record('status', {'state': 'ALIGN', 'reason': 'visual_alignment'})
    recorder.record('command', {'linear_mps': 0., 'angular_rps': -.02})
    recorder.record('status', {'state': 'FAULT', 'reason': 'vision_timeout'})
    recorder.close('FAULT: vision_timeout')
    events = [json.loads(line) for line in (tmp_path / 'run/events.jsonl').read_text().splitlines()]
    assert [e['kind'] for e in events] == ['status', 'command', 'status']
    assert all('unix_s' in e and 'monotonic_s' in e for e in events)
    summary = json.loads((tmp_path / 'run/summary.json').read_text())
    assert summary['last_status']['reason'] == 'vision_timeout'
    assert summary['writer_drained'] and summary['dropped_records'] == 0
    assert (tmp_path / 'run/00_camera.yaml').read_text() == source.read_text()


def test_log_failure_does_not_raise_in_control_callback(tmp_path):
    recorder = DockingRecorder(tmp_path / 'run', {}, [])
    original = recorder.output
    class FullDisk:
        def write(self, value):
            raise OSError('No space left on device')
        def close(self):
            original.close()
    recorder.output = FullDisk()
    recorder.record('status', {'state': 'ALIGN'})
    recorder.close('test')
    assert recorder.error is not None
    # Even if a disk writer fails, future recording remains non-blocking.
    for index in range(3000):
        recorder.record('command', {'index': index})
    assert recorder.dropped > 0

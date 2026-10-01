"""Bounded, asynchronous diagnostics; disk writes never run in control callbacks."""
import hashlib
import json
from pathlib import Path
import queue
import shutil
import threading
import time


class DockingRecorder:
    def __init__(self, directory, config, sources=()):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.queue = queue.Queue(maxsize=2048)
        self.stopping = threading.Event()
        self.dropped = 0
        self.error = None
        self.last_status = None
        manifest = {'started_unix_s': time.time(), 'config': config, 'files': {}}
        for index, source in enumerate(sources):
            source = Path(source)
            if source.is_file():
                name = f'{index:02d}_{source.name}'
                shutil.copy2(source, self.directory / name)
                manifest['files'][str(source)] = {
                    'snapshot': name, 'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
        (self.directory / 'manifest.json').write_text(json.dumps(manifest, indent=2))
        # Fail before motion if the output cannot be opened.
        self.output = (self.directory / 'events.jsonl').open('w', buffering=1)
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def record(self, kind, data):
        if kind == 'status':
            self.last_status = dict(data)
        item = {'kind': kind, 'unix_s': time.time(), 'monotonic_s': time.monotonic(), **data}
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        try:
            while not self.stopping.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                self.output.write(json.dumps(item, allow_nan=False) + '\n')
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.output.close()

    def close(self, exit_reason):
        self.stopping.set()
        self.worker.join(timeout=2)
        summary = {'exit_reason': exit_reason, 'last_status': self.last_status,
                   'dropped_records': self.dropped, 'writer_error': self.error,
                   'writer_drained': not self.worker.is_alive(), 'ended_unix_s': time.time()}
        (self.directory / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))

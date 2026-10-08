"""Bounded readiness buffers and durable IDs; no movement commands are queued."""
import json
import math
import os
import re
import time
from collections import deque
from pathlib import Path


def wait_response(client, request, spin, timeout, label, clock=time.monotonic):
    """Keep one future through the entire response deadline, including late ACKs."""
    deadline = clock() + timeout
    while not client.service_is_ready():
        if clock() >= deadline:
            raise RuntimeError(label + ': service discovery timeout')
        spin(min(.05, max(0., deadline-clock())))
    future = client.call_async(request)
    try:
        while not future.done():
            if clock() >= deadline:
                raise RuntimeError(label + ': response timeout (request retained until deadline)')
            spin(min(.05, max(0., deadline-clock())))
        result = future.result()
        if result is None: raise RuntimeError(label + ': empty response')
        return result
    finally:
        if not future.done():
            client.remove_pending_request(future)
            future.cancel()


def wait_responses(pending, clients, spin, timeout, label, clock=time.monotonic):
    """Keep each already-sent request until ACK/deadline; never resend it."""
    deadline = clock() + timeout
    try:
        while not all(future.done() for future in pending.values()):
            if clock() >= deadline:
                missing = ', '.join(name for name, future in pending.items() if not future.done())
                raise RuntimeError(label + ': response timeout (' + missing + ')')
            spin(min(.05, max(0., deadline-clock())))
        return {name: future.result() for name, future in pending.items()}
    finally:
        for name, future in pending.items():
            if not future.done():
                clients[name].remove_pending_request(future)
                future.cancel()


def lifecycle_evidence_valid(state, pending_since, now):
    """A late read reply is unknown, not Inactive; sensor evidence is separate."""
    if not state or state[0] != 3: return False
    age = now-state[1]
    if 0 <= age <= 3.: return True
    return (pending_since is not None and 0 <= now-pending_since <= 12.
            and 0 <= age <= 15.)


def apply_lifecycle_reply(states, events, name, value, sent, received):
    """Never overwrite a newer transition event with an older RPC snapshot."""
    if events.get(name, -1.) > sent: return False
    states[name] = (value, received)
    return True


class Feedback:
    """A short ring buffer, not a FIFO for replaying stale measurements."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.odom = deque(maxlen=64)
        self.sensor = None

    def odometry(self, age, v, w):
        now = self.clock()
        if not all(math.isfinite(x) for x in (age, v, w)) or not -.1 <= age <= .3:
            self.odom.clear()
            return
        if self.odom and now-self.odom[-1][0] > .3: self.odom.clear()
        self.odom.append((now, abs(v) <= .01 and abs(w) <= .04))

    def motor(self, torque, age):
        self.sensor = (self.clock(), bool(torque), age)

    def stopped(self):
        now = self.clock()
        if not self.odom or now-self.odom[-1][0] > .3: return False
        recent = [s for s in self.odom if now-s[0] <= .8]
        return (len(recent) >= 2 and recent[-1][0]-recent[0][0] >= .5
                and all(s[1] for s in recent))

    def torque(self):
        return (self.sensor is not None and self.clock()-self.sensor[0] <= .5
                and self.sensor[1] and -.1 <= self.sensor[2] <= .5)


class Journal:
    def __init__(self, path, boot):
        self.path, self.boot = Path(path), boot
        try: self.records = json.loads(self.path.read_text())
        except (OSError, ValueError): self.records = {}
        if not isinstance(self.records, dict): self.records = {}

    def get(self, identity):
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,96}', identity):
            raise ValueError('Invalid preparation request ID')
        return self.records.get(identity)

    def put(self, identity, **fields):
        self.get(identity)
        record = dict(self.records.get(identity, {}), **fields,
                      request_id=identity, boot=self.boot, updated_unix=time.time())
        self.records[identity] = record
        # Bounded records retain running entries; no mission/velocity queue.
        finished = sorted((k for k,v in self.records.items() if v.get('status') in ('success','failed')),
                          key=lambda k:self.records[k].get('updated_unix',0))
        for key in finished[:-32]: self.records.pop(key, None)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix('.tmp')
        with temp.open('w') as stream:
            stream.write(json.dumps(self.records, ensure_ascii=False, indent=2))
            stream.flush();os.fsync(stream.fileno())
        temp.replace(self.path)
        return record

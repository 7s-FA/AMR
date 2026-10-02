#!/usr/bin/env python3
"""Reuse only the Burger1 waypoint ROS connection, not routes or motion state."""
import argparse
import codecs
import contextlib
import hashlib
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import socket
import sys
import threading
import time


class ServiceLease:
    """Keep DDS discovery, while discarding each call's unfinished read requests."""
    def __init__(self, client):
        self.client = client
        self.pending = []

    def __getattr__(self, name):
        return getattr(self.client, name)

    def call_async(self, request):
        future = self.client.call_async(request)
        self.pending.append(future)
        return future

    def release(self):
        for future in self.pending:
            if not future.done():
                self.client.remove_pending_request(future)
                future.cancel()
        self.pending.clear()


def prepared_navigator_class(base, endpoints):
    """Only reuse specified service transports. Responses are NEVER cached."""
    class PreparedNavigator(base):
        def __init__(self, **kwargs):
            self.prepared_services = {}
            super().__init__(**kwargs)
            for service_type, name in endpoints:
                self.create_client(service_type, name)

        def create_client(self, service_type, name, *args, **kwargs):
            key = (service_type, name)
            if key not in endpoints:
                return super().create_client(service_type, name, *args, **kwargs)
            if key not in self.prepared_services:
                self.prepared_services[key] = ServiceLease(
                    super().create_client(service_type, name, *args, **kwargs))
            return self.prepared_services[key]

        def destroy_client(self, client):
            if isinstance(client, ServiceLease):
                client.release()
                return True
            return super().destroy_client(client)
    return PreparedNavigator


def validate_request(request, now):
    if request.get('op') not in ('run', 'probe', 'verify'):
        raise ValueError('Unknown waypoint operation')
    issued = request.get('issued')
    if isinstance(issued, bool) or not isinstance(issued, (int, float)) or not 0 <= now-issued <= 2:
        raise ValueError('Expired request; no queued motion')
    if request['op'] == 'run':
        argv = request.get('argv')
        if not isinstance(argv, list) or len(argv) > 100 or not all(isinstance(x, str) for x in argv):
            raise ValueError('Invalid waypoint arguments')
        if '--namespace' not in argv or argv[argv.index('--namespace')+1] != 'burger1':
            raise ValueError('Worker belongs to burger1 only')
        if argv.count('--namespace') != 1 or any(x.startswith('--namespace=') for x in argv):
            raise ValueError('Ambiguous namespace')


def signature(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def monitor_disconnect(connection, finished, interrupt):
    while not finished.wait(.05):
        try:
            readable, _, _ = select.select([connection], [], [], 0)
            if readable and connection.recv(1, socket.MSG_PEEK) == b'':
                interrupt(); return
        except OSError:
            if not finished.is_set(): interrupt()
            return


@contextlib.contextmanager
def forwarded_output(connection, lock):
    """Forward Python and rcutils logs; keep the same logs in the service journal."""
    sys.stdout.flush(); sys.stderr.flush()
    saved_out, saved_err = os.dup(1), os.dup(2)
    reader, writer = os.pipe()
    def drain():
        decoder = codecs.getincrementaldecoder('utf-8')('replace')
        while True:
            raw = os.read(reader, 65536)
            if not raw: break
            os.write(saved_err, raw)
            text = decoder.decode(raw)
            if text:
                try:
                    with lock: connection.sendall((json.dumps({'type': 'log', 'text': text})+'\n').encode())
                except OSError: pass  # The disconnect monitor cancels the route.
    thread = threading.Thread(target=drain, daemon=True); thread.start()
    os.dup2(writer, 1); os.dup2(writer, 2); os.close(writer)
    try: yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(saved_out, 1); os.dup2(saved_err, 2)
        thread.join(timeout=3)
        if thread.is_alive():
            raise RuntimeError('Waypoint log reader did not finish')
        os.close(reader); os.close(saved_out); os.close(saved_err)


def execute_request(module, nav, request, connection):
    lock = threading.Lock(); finished = threading.Event()
    watcher = threading.Thread(target=monitor_disconnect,
        args=(connection, finished, lambda: os.kill(os.getpid(), signal.SIGUSR1)), daemon=True)
    connection.sendall(b'{"type":"accepted"}\n')
    watcher.start()
    try:
        with forwarded_output(connection, lock):
            return module.main(argv=['nav2_waypoints', *request['argv']], prepared_nav=nav)
    finally:
        finished.set(); watcher.join(timeout=.2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--entry', default=os.environ.get('BURGER_WAYPOINT_ENTRY_PATH'))
    a = p.parse_args()
    if not a.entry: raise ValueError('BURGER_WAYPOINT_ENTRY_PATH is required')
    started = time.monotonic()
    loader = importlib.machinery.SourceFileLoader('burger1_waypoint_program', a.entry)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec); loader.exec_module(module)
    if 'prepared_nav' not in __import__('inspect').signature(module.main).parameters:
        raise RuntimeError('Waypoint entry does not support an explicit reusable navigator')
    code_hash = signature(a.entry)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    from nav2_msgs.srv import ManageLifecycleNodes
    rclpy.init(args=['--ros-args', '-r', '/tf:=/burger1/tf', '-r', '/tf_static:=/burger1/tf_static'],
                signal_handler_options=SignalHandlerOptions.NO)
    endpoints = {(GetState, name+'/get_state') for name in
                 ('amcl', 'bt_navigator', 'controller_server', 'velocity_smoother', 'collision_monitor')}
    endpoints.update({(GetParameters, name+'/get_parameters') for name in
                      ('controller_server', 'behavior_server')})
    endpoints.add((ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes'))
    nav = prepared_navigator_class(module.WaypointNavigator, endpoints)(namespace='burger1')
    owner = {'mode': None, 'received': 0.}
    nav.create_subscription(String, 'motion_owner/status',
        lambda msg: owner.update(mode=msg.data, received=time.monotonic()), qos_profile_sensor_data)
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/run/user/'+str(os.getuid())))
    address = runtime/'burger1-waypoint-ready.sock'; meta = address.with_suffix('.json')
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stopping = False; executing = False
    def stop(*_):
        nonlocal stopping
        stopping = True
        if executing: raise KeyboardInterrupt
    def disconnected(*_):
        if executing: raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGUSR1, disconnected)
    try:
        address.unlink(missing_ok=True); server.bind(str(address)); os.chmod(address, 0o600)
        server.listen(1); server.setblocking(False)
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        meta.write_text(json.dumps({'pid': os.getpid(), 'boot': boot, 'entry': a.entry,
                                   'sha256': code_hash, 'startup_s': time.monotonic()-started}))
        print('Burger1 waypoint connection prepared; no movement sent', flush=True)
        while rclpy.ok() and not stopping:
            rclpy.spin_once(nav, timeout_sec=.05)
            try: connection, _ = server.accept()
            except BlockingIOError: continue
            with connection:
                connection.settimeout(2)
                accepted = False
                try:
                    request = json.loads(connection.makefile('rb').readline(16384))
                    validate_request(request, time.monotonic())
                    if signature(a.entry) != code_hash:
                        raise RuntimeError('Waypoint code changed; restart prepared worker')
                    if request['op'] in ('probe', 'verify'):
                        if request['op'] == 'verify':
                            # Diagnostic reads only: no goal, initial pose or speed command.
                            for name in ('amcl', 'bt_navigator'):
                                client = nav.create_client(GetState, name+'/get_state')
                                try:
                                    if not client.wait_for_service(timeout_sec=3):
                                        raise RuntimeError(name+' lifecycle connection missing')
                                    future = client.call_async(GetState.Request())
                                    rclpy.spin_until_future_complete(nav, future, timeout_sec=3)
                                    if (not future.done() or future.result() is None
                                            or future.result().current_state.label != 'active'):
                                        raise RuntimeError(name+' is not confirmed active')
                                finally:
                                    nav.destroy_client(client)
                            nav.verify_controllers({'forward'})
                            nav.verify_terminal_behavior()
                        result = {'type': 'result', 'code': 0, 'owner': owner,
                                  'namespace': nav.get_namespace(), 'motion_sent': False}
                    else:
                        if '--dry-run' not in request['argv']:
                            # Allow the just-confirmed ownership message to arrive; never bypass it.
                            end = time.monotonic()+1.
                            while (owner['mode'] != 'nav' or time.monotonic()-owner['received'] > .8) and time.monotonic() < end:
                                rclpy.spin_once(nav, timeout_sec=.02)
                            if owner['mode'] != 'nav' or time.monotonic()-owner['received'] > .8:
                                raise RuntimeError('Fresh nav ownership required')
                        # Retain only fresh sensor/TF connections; route parameters
                        # are reread by the unchanged waypoint main on EVERY call.
                        nav.goal_handle = nav.result_future = nav.feedback = nav.status = None
                        nav.plan_message = nav.plan_received_at = None; nav.plan_points = ()
                        connection.settimeout(5); executing = accepted = True
                        code = execute_request(module, nav, request, connection)
                        executing = False
                        result = {'type': 'result', 'code': int(code or 0)}
                    connection.sendall((json.dumps(result)+'\n').encode())
                except BaseException as exc:
                    executing = False
                    if accepted:
                        try: module.cancel_and_wait(nav)
                        except BaseException as cancel_error:
                            print('Waypoint cancel failed: '+str(cancel_error), flush=True)
                            stopping = True
                    print('Prepared waypoint request ended: '+str(exc), flush=True)
                    result = {'type': 'result', 'code': 130 if isinstance(exc, KeyboardInterrupt) else 1,
                              'error': str(exc), 'accepted': accepted}
                    try: connection.sendall((json.dumps(result)+'\n').encode())
                    except OSError: pass
                    if isinstance(exc, SystemExit) and not accepted: continue
    finally:
        meta.unlink(missing_ok=True); address.unlink(missing_ok=True); server.close()
        nav.tf_listener.unregister(); nav.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()

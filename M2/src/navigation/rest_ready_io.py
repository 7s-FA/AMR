import codecs,contextlib,json,os,select,socket,sys,threading

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



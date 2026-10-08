"""Read-before-recover localization readiness; no motor or pose commands."""
import time


def ensure_active(query, manager_active, failed_bringup, recover,
                  timeout=90., clock=time.monotonic, sleep=time.sleep, report=print):
    """Only recover an explicitly aborted bringup, once, from known stable states.

    Missing replies are unknown, never proof of failure or permission to reset.
    query and manager_active consume the supplied remaining response budget.
    """
    deadline = clock() + timeout
    recovered = False
    stable = None
    since = clock()
    last_report = None
    states = {}
    last_error = ''
    while clock() < deadline:
        try:
            states = query(min(12., max(.01, deadline-clock())))
            pair = (states.get('map_server'), states.get('amcl'))
            if pair != stable:
                stable, since = pair, clock()
            if pair != last_report:
                report('localization states: map_server=' + str(pair[0]) + ', amcl=' + str(pair[1]))
                last_report = pair
            if pair == (3, 3):
                if manager_active(min(6., max(.01, deadline-clock()))):
                    return states
            elif pair in ((1, 1), (2, 2), (3, 2), (2, 1), (3, 1)) and clock()-since >= 1.:
                if not recovered and failed_bringup() and not manager_active(min(6., max(.01, deadline-clock()))):
                    # Never replay a possibly delivered state-changing request.
                    recovered = True
                    command = ('resume' if pair == (2, 2) else
                               'startup' if pair == (1, 1) else 'restart')
                    report('aborted localization bringup: one ' + command + ' request; ' +
                           ('only failed localization unit restarted' if command == 'restart' else 'running services retained'))
                    try:
                        recover(command, min(45., max(.01, deadline-clock())))
                    except RuntimeError as exc:
                        # ACK loss is ambiguous. Verify actual state, do not resend.
                        last_error = str(exc)
                        report('localization recovery acknowledgment uncertain: ' + last_error + '; checking actual states')
        except RuntimeError as exc:
            last_error = str(exc)
            report('localization state response delayed: ' + last_error + '; read-only query will retry')
        sleep(min(1., max(0., deadline-clock())))
    raise RuntimeError('Localization readiness deadline: states=' + str(states) + '; ' + last_error)

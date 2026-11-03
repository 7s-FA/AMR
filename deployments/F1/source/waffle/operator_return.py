"""Explicit operator-confirmed direct departure; never classifies a dock as clear."""
import time


def prepare_direct_return(nav, shared, now=None):
    """Wait for empty-fork lowering without requesting backup or starting Nav2."""
    if shared.linear_pallet_exist:
        nav.fail_backup('직접 복귀 전 빈 포크 확인 필요')
        return False
    nav.publish_backup()  # Zero only, including while waiting for the lift.
    if shared.linear_state == 'down' and shared.linear_action == 'stop':
        return True
    now = time.monotonic() if now is None else now
    if getattr(nav, 'direct_lift_started', None) is None:
        nav.direct_lift_started = now
        nav.direct_lift_requested = False
    if now - nav.direct_lift_started > 30:
        shared.linear_action = 'stop'
        nav.fail_backup('직접 복귀 전 포크 하강 시간 초과')
        return False
    if not nav.direct_lift_requested and shared.linear_action == 'stop':
        nav.direct_lift_requested = True
        shared.linear_command_sent_flag = False
        shared.linear_action = 'down'
    return False

"""Lower empty forks after undocking, before travelling between warehouses."""
import time


STORAGE = {'GO_TO_STORAGE_A_01': 'A', 'GO_TO_STORAGE_A_02': 'A',
           'GO_TO_STORAGE_B_01': 'B', 'GO_TO_STORAGE_B_02': 'B'}


def prepare_empty_travel(nav, shared, now=None):
    origin = STORAGE.get(shared.docking_place)
    target = STORAGE.get(shared.navigator_index)
    if not origin or not target or origin == target or shared.linear_pallet_exist:
        return True
    # Never lower while the forks are still inserted into the storage rack.
    if not nav.backup_done:
        return False
    nav.publish_backup()
    if shared.linear_state == 'down' and shared.linear_action == 'stop':
        return True
    now = time.monotonic() if now is None else now
    if getattr(nav, 'travel_lift_started', None) is None:
        nav.travel_lift_started = now
        nav.travel_lift_requested = False
    if now - nav.travel_lift_started > 30.0:
        shared.linear_action = 'stop'
        nav.fail_backup('창고 간 이동 전 빈 포크 하강 시간 초과')
        return False
    if not nav.travel_lift_requested and shared.linear_action == 'stop':
        nav.travel_lift_requested = True
        shared.linear_command_sent_flag = False
        shared.linear_action = 'down'
        nav.get_logger().info('창고 간 이동 전 빈 포크 하강 · 완료 전 Nav2 이동 보류')
    return False

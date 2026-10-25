class Shared_data():
    cmd_WAFFLE_PARK = "GO_TO_PARK"
    cmd_WAFFLE_ASM = "GO_TO_ASM"
    cmd_WAFFLE_A_01 = "GO_TO_STORAGE_A_01"
    cmd_WAFFLE_A_02 = "GO_TO_STORAGE_A_02"
    cmd_WAFFLE_B_01 = "GO_TO_STORAGE_B_01"
    cmd_WAFFLE_B_02 = "GO_TO_STORAGE_B_02"

    linear_state = ""
    linear_action = ""
    linear_preaction = ""
    linear_command_sent_flag = False

    navigator_index = ""
    navigator_start = False
    navigator_end = False
    navigator_end_msg = ""
    
    docking_state = "idle" #jh 메롱
    docking_control_acquired = False
    docking_place = "" # 정제 추가

    host_receive_command = ""
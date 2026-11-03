import os
import time

import rclpy
from rclpy.node import Node

from gpiozero import DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

from waffle.shared_data import Shared_data

CHIP = 4       # gpiodetect의 pinctrl-rp1 칩 번호
GPIO_PIN_A = 20 # GPIO 번호. 물리적 핀 번호는 38번
GPIO_PIN_B = 21

class Linear(Node):
    def __init__(self):
        super().__init__("linear_actuator")
        
        self.delay_timer = 0.0

        Shared_data.linear_state = ""
        Shared_data.linear_preaction = ""
        Shared_data.linear_action = "stop" if os.environ.get("F1_HOST_MANAGED") == "1" else "down"
        
        Shared_data.linear_command_sent_flag = False

        Shared_data.linear_pallet_exist = False

        factory = LGPIOFactory(chip=CHIP)
        self.output_A = DigitalOutputDevice(
            GPIO_PIN_A,
            active_high=True,
            initial_value=False,
            pin_factory=factory,
        )

        self.output_B = DigitalOutputDevice(
            GPIO_PIN_B,
            active_high=True,
            initial_value=False,
            pin_factory=factory,
        )

        self.get_logger().info("linear actuator node 생성")
        self.control_timer = self.create_timer(0.01, self.run)


    def run(self):
        if Shared_data.linear_preaction != Shared_data.linear_action:
            Shared_data.linear_preaction = Shared_data.linear_action
            self.delay_timer = time.monotonic()

            self.get_logger().info(f"새 명령: {Shared_data.linear_action}")

        if  Shared_data.linear_action == "up":
            if  Shared_data.linear_state == "up":
                self.output_A.off()
                self.output_B.off()
                Shared_data.linear_action = "stop"
                Shared_data.linear_command_sent_flag = True

            elif Shared_data.linear_state == "middle":
                if  time.monotonic() >= self.delay_timer + 2.0:
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "up"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()
            elif Shared_data.linear_state == "2nd_floor_down":
                if  time.monotonic() >= self.delay_timer + 3.0:
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "up"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()

            else:
                if  time.monotonic() >= self.delay_timer + 12.0:
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "up"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()
        
        elif Shared_data.linear_action == "middle":
            if  Shared_data.linear_state == "up":
                if  time.monotonic() >= self.delay_timer + 1.0:
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "middle"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.off()
                    self.output_A.on()

            elif Shared_data.linear_state == "middle":
                self.output_A.off()
                self.output_B.off()
                Shared_data.linear_action = "stop"
                Shared_data.linear_command_sent_flag = True

            elif Shared_data.linear_state == "down":
                if  time.monotonic() >= self.delay_timer + 11.0:
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "middle"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True
                else:
                    self.output_A.off()
                    self.output_B.on()           
            else:
                if  time.monotonic() >= self.delay_timer + 12.0:
                    if  time.monotonic() >= self.delay_timer + 13.0: # 27.2 였음
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "middle"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True
                    else:
                        self.output_B.off()
                        self.output_A.on()
                else:
                    self.output_A.off()
                    self.output_B.on()

        elif Shared_data.linear_action == "2nd_floor_down":
            if Shared_data.linear_state == "up":
                if  time.monotonic() >= self.delay_timer + 2.45:
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "2nd_floor_down"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.off()
                    self.output_A.on()

            elif Shared_data.linear_state == "down":
                if  time.monotonic() >= self.delay_timer + 9.55:
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "2nd_floor_down"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()

            elif Shared_data.linear_state == "2nd_floor_down":
                self.output_A.off()
                self.output_B.off()
                Shared_data.linear_action = "stop"
                Shared_data.linear_command_sent_flag = True

            else:
                if  time.monotonic() >= self.delay_timer + 12.0:
                    if  time.monotonic() >= self.delay_timer + 14.45: # 27.2 였음
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "2nd_floor_down"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True
                    else:
                        self.output_B.off()
                        self.output_A.on()
                else:
                    self.output_A.off()
                    self.output_B.on()


        elif Shared_data.linear_action == "1st_floor_up":
            if  Shared_data.linear_state == "1st_floor_down":
                if  time.monotonic() >= self.delay_timer + 1.13:
                    
                    Shared_data.departure_reset_requested = True
                    
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "1st_floor_up"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()

            elif Shared_data.linear_state == "1st_floor_up":
                self.output_A.off()
                self.output_B.off()
                Shared_data.linear_action = "stop"
                Shared_data.linear_command_sent_flag = True

            elif Shared_data.linear_state == "down":
                if  time.monotonic() >= self.delay_timer + 2.23:
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "1st_floor_up"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()
            else:
                if  time.monotonic() >= self.delay_timer + 12.0:
                    if  time.monotonic() >= self.delay_timer + 21.77: # 
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "1st_floor_up"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True
                    else:
                        self.output_B.off()
                        self.output_A.on()
                else:
                    self.output_A.off()
                    self.output_B.on()

        elif Shared_data.linear_action == "1st_floor_down":
            if  Shared_data.linear_state == "1st_floor_up":
                if  time.monotonic() >= self.delay_timer + 1.16:
                    
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "1st_floor_down"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.off()
                    self.output_A.on()

            elif Shared_data.linear_state == "down":
                if  time.monotonic() >= self.delay_timer + 0.9:
                    
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "1st_floor_down"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

                else:
                    self.output_B.on()
                    self.output_A.off()
            elif  Shared_data.linear_state == "1st_floor_down":
                self.output_A.off()
                self.output_B.off()

                Shared_data.linear_action = "stop"
                Shared_data.linear_command_sent_flag = True

            else:
                if  time.monotonic() >= self.delay_timer + 12.0:
                    if  time.monotonic() >= self.delay_timer + 13.1: # 27.2 였음
                        self.output_A.off()
                        self.output_B.off()

                        Shared_data.linear_state = "1st_floor_down"
                        Shared_data.linear_action = "stop"
                        Shared_data.linear_command_sent_flag = True
                    else:
                        self.output_B.on()
                        self.output_A.off()
                else:
                    self.output_A.on()
                    self.output_B.off()

        elif Shared_data.linear_action == "down":
            if  time.monotonic() >= self.delay_timer + 12.0:
                    self.output_A.off()
                    self.output_B.off()

                    Shared_data.linear_state = "down"
                    Shared_data.linear_action = "stop"
                    Shared_data.linear_command_sent_flag = True

            else:
                self.output_B.off()
                self.output_A.on()
            
        else:
            self.output_A.off()
            self.output_B.off()

            if  Shared_data.linear_action != "stop": 
                self.get_logger().info(f"\033[1;31mlinear_action data가 잘못 들어왔습니다. linear_action : {Shared_data.linear_action}\033[0m")

            Shared_data.linear_action = "stop"


        if  Shared_data.linear_command_sent_flag == True:
            self.get_logger().info(f"\033[1;32m{Shared_data.linear_preaction}동작을 완료하였습니다.\033[0m")
            Shared_data.linear_command_sent_flag = False

    def destroy_node(self):
        self.output_A.off()
        self.output_B.off()
        self.output_A.close()
        self.output_B.close()
        return super().destroy_node()

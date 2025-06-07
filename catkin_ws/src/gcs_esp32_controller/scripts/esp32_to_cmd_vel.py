#!/usr/bin/env python3

import rospy
import serial
import json
import math
from geometry_msgs.msg import Twist
from jethexa_controller_interfaces.msg import Traveling

NODE_NAME = 'gcs_esp32_to_cmd_vel_node'

TRIPOD_GAIT = 1
RIPPLE_GAIT = 2
DEFAULT_GAIT = TRIPOD_GAIT
SPEED_THRESHOLD_FOR_RIPPLE = 0.03

current_robot_state = {
    "active_gait": DEFAULT_GAIT,
    "target_gait": DEFAULT_GAIT,
    "step_height": 20.0,
    "step_period": 1.0,
    "is_commanding_movement": False, # GCS가 현재 이동 명령(cmd_vel != 0)을 보내고 있는지
    "stop_initiated": False          # gait=0 (발구름 멈춤) 명령을 보냈는지
}

traveling_pub = None
cmd_vel_pub = None # cmd_vel_pub도 전역으로 선언

def send_traveling_cmd(gait_val, period_val=None, height_val=None, interrupt_val=True, steps_val=0, stride_val=0.0, direction_val=0.0, rotation_val=0.0):
    global traveling_pub, current_robot_state
    if traveling_pub is None:
        rospy.logwarn_throttle(1.0, "Traveling publisher is not initialized yet.")
        return

    msg = Traveling()
    msg.gait = int(gait_val)
    msg.stride = float(stride_val)
    msg.height = float(height_val if height_val is not None else current_robot_state["step_height"])
    msg.direction = float(direction_val)
    msg.rotation = float(rotation_val)
    msg.time = float(period_val if period_val is not None else current_robot_state["step_period"])
    msg.steps = int(steps_val)
    msg.interrupt = bool(interrupt_val)
    msg.relative_height = False

    try:
        traveling_pub.publish(msg)
        # rospy.loginfo(f"Published Traveling: gait={msg.gait}")
    except rospy.ROSException as e:
        rospy.logwarn_throttle(5.0, f"ROS Exception during Traveling publish: {e}.")

def main():
    global traveling_pub, cmd_vel_pub, current_robot_state

    rospy.init_node(NODE_NAME)

    serial_port = rospy.get_param("~serial_port", "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_EC:DA:3B:51:98:C0-if00")
    baud_rate = rospy.get_param("~baud_rate", 115200)
    max_linear_speed = rospy.get_param("~max_linear_speed", 0.08)
    max_angular_speed = rospy.get_param("~max_angular_speed", 0.25)
    
    current_robot_state["step_height"] = rospy.get_param("~default_step_height", current_robot_state["step_height"])
    current_robot_state["step_period"] = rospy.get_param("~default_step_period", current_robot_state["step_period"])
    
    adc_center = 2048
    joystick_deadzone = 100
    effective_adc_range = adc_center - joystick_deadzone

    cmd_vel_pub = rospy.Publisher('/jethexa/jethexa_controller/cmd_vel', Twist, queue_size=10)
    traveling_pub = rospy.Publisher('/jethexa/jethexa_controller/traveling', Traveling, queue_size=10)

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=0.1) # timeout을 짧게 설정
        rospy.loginfo(f"Successfully connected to ESP32 on {serial_port}")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect to ESP32 on {serial_port}: {e}")
        return

    rate = rospy.Rate(50) # ESP32 전송 빈도와 유사하게 설정

    # 초기 gait 파라미터 설정 (선택적, 컨트롤러 기본값 사용 시 주석 처리)
    # send_traveling_cmd(gait_val=(10 + current_robot_state["active_gait"]),
    #                    period_val=current_robot_state["step_period"],
    #                    height_val=current_robot_state["step_height"])
    # rospy.sleep(0.1) # 파라미터 적용 대기

    while not rospy.is_shutdown():
        line_processed_in_loop = False
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                if not line:
                    rate.sleep()
                    continue
                
                line_processed_in_loop = True # 이번 루프에서 새 데이터를 처리했음
                data = json.loads(line)

                left_y_adc = data['left']['y']
                left_x_adc = data['left']['x']
                right_x_adc = data['right']['x']

                # --- ADC to Speed Conversion ---
                y_offset = left_y_adc - adc_center
                linear_x = 0.0
                if abs(y_offset) >= joystick_deadzone:
                    normalized_offset = (y_offset - (joystick_deadzone if y_offset > 0 else -joystick_deadzone)) / effective_adc_range
                    linear_x = max_linear_speed * normalized_offset
                    linear_x *= 1 

                x_offset_left = left_x_adc - adc_center
                linear_y = 0.0
                if abs(x_offset_left) >= joystick_deadzone:
                    normalized_offset = (x_offset_left - (joystick_deadzone if x_offset_left > 0 else -joystick_deadzone)) / effective_adc_range
                    linear_y = max_linear_speed * normalized_offset
                    linear_y *= 1 

                x_offset_right = right_x_adc - adc_center
                angular_z = 0.0
                if abs(x_offset_right) >= joystick_deadzone:
                    normalized_offset = (x_offset_right - (joystick_deadzone if x_offset_right > 0 else -joystick_deadzone)) / effective_adc_range
                    angular_z = max_angular_speed * normalized_offset
                    angular_z *= 1
                # --- End ADC to Speed Conversion ---

                is_current_cmd_vel_zero = abs(linear_x) < 0.001 and abs(linear_y) < 0.001 and abs(angular_z) < 0.001

                if is_current_cmd_vel_zero:
                    if not current_robot_state["stop_initiated"]:
                        rospy.loginfo("CMD_VEL is zero. Sending command to stop all movements (gait 0).")
                        send_traveling_cmd(gait_val=0, period_val=current_robot_state["step_period"])
                        current_robot_state["is_commanding_movement"] = False
                        current_robot_state["stop_initiated"] = True # 발구름 멈춤 명령을 보냈음을 표시
                    # cmd_vel (0,0,0)을 한 번 더 보내거나, 안 보낼 수도 있음 (gait 0이 우선)
                    # twist_msg = Twist()
                    # cmd_vel_pub.publish(twist_msg)
                else: # 이동 명령이 있는 경우
                    current_robot_state["stop_initiated"] = False # 이동 시작 시 멈춤 명령 플래그 리셋
                    
                    if not current_robot_state["is_commanding_movement"]:
                        # 이전에 멈춰있다가 다시 움직이기 시작하는 경우, gait 파라미터를 다시 설정해줄 수 있음
                        # (컨트롤러가 gait=0 이후에 cmd_gait를 유지하는지, 아니면 기본값으로 돌아가는지에 따라 필요성 결정)
                        rospy.loginfo("Movement resumed. Ensuring gait parameters are set.")
                        send_traveling_cmd(gait_val=(10 + current_robot_state["active_gait"]), # 현재 active_gait 사용
                                           period_val=current_robot_state["step_period"],
                                           height_val=current_robot_state["step_height"],
                                           steps_val=0)
                        # rospy.sleep(0.05) # 파라미터 적용 시간

                    current_robot_state["is_commanding_movement"] = True
                    linear_speed_magnitude = math.sqrt(linear_x**2 + linear_y**2)
                    
                    target_gait_for_current_speed = TRIPOD_GAIT
                    if linear_speed_magnitude < SPEED_THRESHOLD_FOR_RIPPLE and abs(angular_z) < (max_angular_speed * 0.3):
                        target_gait_for_current_speed = RIPPLE_GAIT
                    
                    if current_robot_state["active_gait"] != target_gait_for_current_speed:
                        rospy.loginfo(f"Speed: {linear_speed_magnitude:.3f}. Target: {target_gait_for_current_speed}. Active: {current_robot_state['active_gait']}. Changing gait.")
                        send_traveling_cmd(gait_val=(10 + target_gait_for_current_speed),
                                           period_val=current_robot_state["step_period"],
                                           height_val=current_robot_state["step_height"],
                                           steps_val=0)
                        current_robot_state["active_gait"] = target_gait_for_current_speed
                        # rospy.sleep(0.05) # 파라미터 적용 시간

                    twist_msg = Twist()
                    twist_msg.linear.x = linear_x
                    twist_msg.linear.y = linear_y
                    twist_msg.angular.z = angular_z
                    cmd_vel_pub.publish(twist_msg)

            except json.JSONDecodeError:
                rospy.logwarn_throttle(1.0, f"JSON Decode Error. Received: {line if 'line' in locals() else 'N/A'}")
            except KeyError as e:
                rospy.logwarn_throttle(1.0, f"KeyError: {e}. Received JSON: {data if 'data' in locals() else 'N/A'}")
            except serial.SerialException as e:
                rospy.logerr(f"Serial communication error: {e}")
                if ser: ser.close()
                rospy.signal_shutdown("Serial error, shutting down.")
                break
            except Exception as e:
                rospy.logerr_throttle(1.0, f"An error occurred in main loop: {e}")
        
        # 만약 이번 루프에서 시리얼 데이터를 처리하지 않았고 (ser.in_waiting == 0),
        # 그리고 현재 로봇이 이동 중이 아니라고 GCS가 판단하고 있다면 (stop_initiated == True),
        # cmd_vel을 계속 발행할 필요가 없음.
        # 하지만, 만약 로봇 컨트롤러가 cmd_vel (0,0,0)을 계속 받아야만 완전히 멈춘 상태를 유지한다면,
        # 아래 로직은 필요함. JetHexa의 경우 gait=0 명령으로 충분할 것으로 예상.
        # if not line_processed_in_loop and current_robot_state["stop_initiated"]:
        #     # 안전을 위해 cmd_vel (0,0,0)을 주기적으로 발행할 수 있으나,
        #     # gait=0 명령이 제대로 동작한다면 불필요할 수 있음.
        #     # 여기서는 일단 추가 발행 안 함.
        #     pass

        rate.sleep()

    if ser and ser.is_open:
        ser.close()
    rospy.loginfo("Serial port closed. Exiting.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Node '{NODE_NAME}' interrupted.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main of '{NODE_NAME}': {e}")
    finally:
        rospy.loginfo(f"Script '{NODE_NAME}' finished.")

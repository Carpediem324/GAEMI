#!/usr/bin/env python3

import rospy
import serial
import json
import math
import socket
from geometry_msgs.msg import Twist
from jethexa_controller_interfaces.msg import Traveling
from std_msgs.msg import String, Int32

NODE_NAME = 'gcs_esp32_to_cmd_vel_node'

# Gait definitions
TRIPOD_GAIT = 1
RIPPLE_GAIT = 2
DEFAULT_GAIT = TRIPOD_GAIT
SPEED_THRESHOLD_FOR_RIPPLE = 0.03

# Operation modes
MANUAL_MODE = 'manual'
POLICY_MODE = 'policy'
AUTO_MODE = 'auto'
current_operation_mode = MANUAL_MODE

# Robot state
current_robot_state = {
    "active_gait": DEFAULT_GAIT,
    "target_gait": DEFAULT_GAIT,
    "step_height": 20.0,
    "step_period": 1.0,
    "is_commanding_movement": False, # GCS가 현재 이동 명령(cmd_vel != 0)을 보내고 있는지
    "stop_initiated": False          # gait=0 (발구름 멈춤) 명령을 보냈는지
}

# Publishers (initialized in main)
traveling_pub = None
cmd_vel_pub = None  # cmd_vel_pub도 전역으로 선언
current_mode_pub = None  # 현재 모드 발행을 위한 Publisher


def mode_callback(msg):
    rospy.loginfo_throttle(1.0, "mode_callback received a message!") # 호출 확인용 로그 추가
    global current_operation_mode, current_mode_pub
    mode_val = msg.data
    prev_mode = current_operation_mode

    if mode_val == 0:
        current_operation_mode = MANUAL_MODE
    elif mode_val == 1:
        current_operation_mode = POLICY_MODE
    elif mode_val == 2:
        current_operation_mode = AUTO_MODE
    else:
        rospy.logwarn(f"Received unknown mode value: {mode_val}. Keeping mode: {current_operation_mode}")
        return

    if prev_mode != current_operation_mode:
        rospy.loginfo(f"Operation mode changed from {prev_mode} to {current_operation_mode}")
        current_mode_pub.publish(current_operation_mode)

    # 모드 변경 시 로봇 정지
    if prev_mode != current_operation_mode and current_operation_mode != MANUAL_MODE:
        rospy.loginfo(f"Mode changed to {current_operation_mode}. Stopping robot.")
        send_traveling_cmd(gait_val=0)  # 모든 움직임 정지
        twist_msg = Twist()
        if cmd_vel_pub:  # cmd_vel_pub이 초기화되었는지 확인
            cmd_vel_pub.publish(twist_msg)
        current_robot_state["is_commanding_movement"] = False
        current_robot_state["stop_initiated"] = True


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
    global traveling_pub, cmd_vel_pub, current_robot_state, current_mode_pub, current_operation_mode

    rospy.init_node(NODE_NAME)

    # params
    POLICY_SERVER_IP   = rospy.get_param("~policy_server_ip",   "192.168.100.119")
    POLICY_SERVER_PORT = rospy.get_param("~policy_server_port", 9997)
    serial_port        = rospy.get_param("~serial_port",        "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_EC:DA:3B:51:98:C0-if00")
    baud_rate          = rospy.get_param("~baud_rate",          115200)
    max_linear_speed   = rospy.get_param("~max_linear_speed",   0.08)
    max_angular_speed  = rospy.get_param("~max_angular_speed",  0.25)
    current_robot_state["step_height"] = rospy.get_param("~default_step_height", current_robot_state["step_height"])
    current_robot_state["step_period"] = rospy.get_param("~default_step_period", current_robot_state["step_period"])

    # joystick ADC params
    adc_center = 2048
    joystick_deadzone = 100
    effective_adc_range = adc_center - joystick_deadzone

    # connect to policy server socket
    try:
        policy_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        policy_sock.connect((POLICY_SERVER_IP, POLICY_SERVER_PORT))
        rospy.loginfo(f"[main] Connected to policy server at {POLICY_SERVER_IP}:{POLICY_SERVER_PORT}")
    except Exception as e:
        rospy.logwarn(f"[main] Could not connect to policy server: {e}")
        policy_sock = None

    # publishers & subscriber
    cmd_vel_pub = rospy.Publisher('/jethexa/jethexa_controller/cmd_vel', Twist, queue_size=10)
    traveling_pub = rospy.Publisher('/jethexa/jethexa_controller/traveling', Traveling, queue_size=10)
    current_mode_pub = rospy.Publisher('/gcs/current_mode', String, queue_size=10, latch=True)

    # Mode subscriber
    rospy.Subscriber('/gcs/robot_command/mode', Int32, mode_callback)
    
    # 초기 모드 발행
    current_mode_pub.publish(current_operation_mode)

    # connect to ESP32 serial
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1) # timeout을 0.1에서 1로 늘려봄
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
        # default speeds
        linear_x = linear_y = angular_z = 0.0

        # read joystick
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                data = json.loads(line)
                # extract ADC
                ly = data['left']['y'] - adc_center
                lx = data['left']['x'] - adc_center
                rx = data['right']['x'] - adc_center
                # convert
                if abs(ly) >= joystick_deadzone:
                    norm = (ly - math.copysign(joystick_deadzone, ly)) / effective_adc_range
                    linear_x = max_linear_speed * norm
                if abs(lx) >= joystick_deadzone:
                    norm = (lx - math.copysign(joystick_deadzone, lx)) / effective_adc_range
                    linear_y = max_linear_speed * norm
                if abs(rx) >= joystick_deadzone:
                    norm = (rx - math.copysign(joystick_deadzone, rx)) / effective_adc_range
                    angular_z = max_angular_speed * norm
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"[main] joystick parse error: {e}")

        # mode-specific behavior
        if current_operation_mode == MANUAL_MODE:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').rstrip()
                    if not line:
                        rate.sleep()
                        continue
                    
                    line_processed_in_loop = True
                    data = json.loads(line)

                    # 데이터 구조 검증 및 값 추출
                    if not (
                        isinstance(data, dict) and \
                        'left' in data and isinstance(data.get('left'), dict) and \
                        'x' in data.get('left', {}) and 'y' in data.get('left', {}) and \
                        'right' in data and isinstance(data.get('right'), dict) and \
                        'x' in data.get('right', {})
                    ):
                        rospy.logwarn_throttle(1.0, f"MANUAL_MODE: Received JSON does not have expected structure. Data: {data}")
                        rate.sleep()
                        continue

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
                            rospy.loginfo("MANUAL_MODE: CMD_VEL is zero. Sending command to stop all movements (gait 0).")
                            send_traveling_cmd(gait_val=0, period_val=current_robot_state["step_period"])
                            current_robot_state["is_commanding_movement"] = False
                            current_robot_state["stop_initiated"] = True # 발구름 멈춤 명령을 보냈음을 표시
                    else: # 이동 명령이 있는 경우
                        current_robot_state["stop_initiated"] = False # 이동 시작 시 멈춤 명령 플래그 리셋
                        
                        if not current_robot_state["is_commanding_movement"]:
                            rospy.loginfo("MANUAL_MODE: Movement resumed. Ensuring gait parameters are set.")
                            send_traveling_cmd(gait_val=(10 + current_robot_state["active_gait"]), 
                                               period_val=current_robot_state["step_period"],
                                               height_val=current_robot_state["step_height"],
                                               steps_val=0)

                        current_robot_state["is_commanding_movement"] = True
                        linear_speed_magnitude = math.sqrt(linear_x**2 + linear_y**2)
                        
                        target_gait_for_current_speed = TRIPOD_GAIT
                        if linear_speed_magnitude < SPEED_THRESHOLD_FOR_RIPPLE and abs(angular_z) < (max_angular_speed * 0.3):
                            target_gait_for_current_speed = RIPPLE_GAIT
                        
                        if current_robot_state["active_gait"] != target_gait_for_current_speed:
                            rospy.loginfo(f"MANUAL_MODE: Speed: {linear_speed_magnitude:.3f}. Target: {target_gait_for_current_speed}. Active: {current_robot_state['active_gait']}. Changing gait.")
                            send_traveling_cmd(gait_val=(10 + target_gait_for_current_speed),
                                               period_val=current_robot_state["step_period"],
                                               height_val=current_robot_state["step_height"],
                                               steps_val=0)
                            current_robot_state["active_gait"] = target_gait_for_current_speed

                        twist_msg = Twist()
                        twist_msg.linear.x = linear_x
                        twist_msg.linear.y = linear_y
                        twist_msg.angular.z = angular_z
                        cmd_vel_pub.publish(twist_msg)

                except json.JSONDecodeError:
                    rospy.logwarn_throttle(1.0, f"MANUAL_MODE: JSON Decode Error. Received raw line: {line if 'line' in locals() else 'N/A'}")
                except KeyError as e: # 이 KeyError는 위의 data.get() 사용으로 인해 거의 발생하지 않을 것으로 예상됨
                    rospy.logwarn_throttle(1.0, f"MANUAL_MODE: KeyError: {e}. This should not happen if structure check is correct. Data: {data if 'data' in locals() else 'N/A'}")
                except serial.SerialException as e:
                    rospy.logerr(f"MANUAL_MODE: Serial communication error: {e}")
                    if ser: ser.close()
                    rospy.signal_shutdown("Serial error, shutting down.")
                    break
                except Exception as e:
                    rospy.logerr_throttle(1.0, f"MANUAL_MODE: An error occurred in main loop: {e}")

        elif current_operation_mode == POLICY_MODE:
            # POLICY 모드 로직 (현재 비워둠)
            # 이 모드에서는 cmd_vel을 발행하지 않음
            # 만약 POLICY 모드 진입 시 로봇을 정지시키고 싶다면 mode_callback에 로직 추가
            if not line_processed_in_loop and not current_robot_state["stop_initiated"]:
                 # POLICY 모드로 전환되었지만 아직 로봇이 움직이고 있을 수 있는 경우, 정지 명령을 내릴 수 있습니다.
                 # 혹은, 이전에 manual 모드에서 움직임 명령을 보내던 상태였다면 정지 명령을 보냅니다.
                 rospy.loginfo_throttle(1.0, f"POLICY_MODE: Stopping robot if it was moving.")
                 send_traveling_cmd(gait_val=0) # 모든 움직임 정지
                 twist_msg = Twist()
                 if cmd_vel_pub: cmd_vel_pub.publish(twist_msg) # (0,0,0) 발행
                 current_robot_state["is_commanding_movement"] = False
                 current_robot_state["stop_initiated"] = True
            # always send current joystick speeds to policy server
            payload = {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z}
            if policy_sock:
                try:
                    policy_sock.sendall((json.dumps(payload) + "\n").encode('utf-8'))
                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"[POLICY_MODE] send error: {e}")
            else:
                rospy.logwarn_throttle(5.0, "[POLICY_MODE] no policy socket")

        elif current_operation_mode == AUTO_MODE:
            # AUTO 모드 로직 (현재 비워둠)
            # 이 모드에서는 cmd_vel을 발행하지 않음
            # 만약 AUTO 모드 진입 시 로봇을 정지시키고 싶다면 mode_callback에 로직 추가
            if not line_processed_in_loop and not current_robot_state["stop_initiated"]:
                 rospy.loginfo_throttle(1.0, f"AUTO_MODE: Stopping robot if it was moving.")
                 send_traveling_cmd(gait_val=0) # 모든 움직임 정지
                 twist_msg = Twist()
                 if cmd_vel_pub: cmd_vel_pub.publish(twist_msg) # (0,0,0) 발행
                 current_robot_state["is_commanding_movement"] = False
                 current_robot_state["stop_initiated"] = True
            pass
        
        # 루프 마지막에 현재 모드와 상태를 로깅 (디버깅용, 필요시 주기 조절)
        # rospy.loginfo_throttle(5.0, f"Current Mode: {current_operation_mode}, Commanding: {current_robot_state['is_commanding_movement']}, Stop Initiated: {current_robot_state['stop_initiated']}")

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
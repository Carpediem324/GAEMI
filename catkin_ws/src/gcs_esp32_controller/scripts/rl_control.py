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
    "step_height": 20.0,
    "step_period": 1.0,
    "is_commanding_movement": False,
    "stop_initiated": False
}

# Publishers (initialized in main)
traveling_pub = None
cmd_vel_pub = None
current_mode_pub = None

def mode_callback(msg):
    global current_operation_mode
    prev = current_operation_mode
    if msg.data == 0:
        new = MANUAL_MODE
    elif msg.data == 1:
        new = POLICY_MODE
    elif msg.data == 2:
        new = AUTO_MODE
    else:
        rospy.logwarn(f"[mode_callback] Unknown mode {msg.data}, keeping {current_operation_mode}")
        return

    if new != prev:
        current_operation_mode = new
        rospy.loginfo(f"[mode_callback] Mode changed: {prev} -> {new}")
        current_mode_pub.publish(current_operation_mode)
        # on mode change, stop robot immediately
        send_traveling_cmd(gait_val=0)
        twist = Twist()
        cmd_vel_pub.publish(twist)
        current_robot_state["is_commanding_movement"] = False
        current_robot_state["stop_initiated"] = True

def send_traveling_cmd(gait_val, period_val=None, height_val=None,
                       interrupt_val=True, steps_val=0,
                       stride_val=0.0, direction_val=0.0,
                       rotation_val=0.0):
    if traveling_pub is None:
        rospy.logwarn_throttle(1.0, "[send_traveling_cmd] publisher not ready")
        return
    msg = Traveling()
    msg.gait = int(gait_val)
    msg.stride = float(stride_val)
    msg.height = float(height_val or current_robot_state["step_height"])
    msg.direction = float(direction_val)
    msg.rotation = float(rotation_val)
    msg.time = float(period_val or current_robot_state["step_period"])
    msg.steps = int(steps_val)
    msg.interrupt = bool(interrupt_val)
    msg.relative_height = False
    traveling_pub.publish(msg)

def main():
    global traveling_pub, cmd_vel_pub, current_mode_pub

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
    current_mode_pub = rospy.Publisher('/gcs/current_mode', String, queue_size=1, latch=True)
    rospy.Subscriber('/gcs/robot_command/mode', Int32, mode_callback)
    current_mode_pub.publish(current_operation_mode)

    # connect to ESP32 serial
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
        rospy.loginfo(f"[main] Connected to ESP32 on {serial_port}")
    except serial.SerialException as e:
        rospy.logerr(f"[main] Serial error: {e}")
        return

    # joystick ADC params
    adc_center = 2048
    joystick_deadzone = 100
    effective_range = adc_center - joystick_deadzone

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
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
                    norm = (ly - math.copysign(joystick_deadzone, ly)) / effective_range
                    linear_x = max_linear_speed * norm
                if abs(lx) >= joystick_deadzone:
                    norm = (lx - math.copysign(joystick_deadzone, lx)) / effective_range
                    linear_y = max_linear_speed * norm
                if abs(rx) >= joystick_deadzone:
                    norm = (rx - math.copysign(joystick_deadzone, rx)) / effective_range
                    angular_z = max_angular_speed * norm
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"[main] joystick parse error: {e}")

        # mode-specific behavior
        if current_operation_mode == MANUAL_MODE:
            is_zero = abs(linear_x) < 1e-3 and abs(linear_y) < 1e-3 and abs(angular_z) < 1e-3
            if is_zero:
                if not current_robot_state["stop_initiated"]:
                    send_traveling_cmd(gait_val=0)
                    cmd_vel_pub.publish(Twist())
                    current_robot_state["is_commanding_movement"] = False
                    current_robot_state["stop_initiated"] = True
            else:
                if not current_robot_state["is_commanding_movement"]:
                    send_traveling_cmd(
                        gait_val=10 + current_robot_state["active_gait"],
                        period_val=current_robot_state["step_period"],
                        height_val=current_robot_state["step_height"]
                    )
                    current_robot_state["is_commanding_movement"] = True
                    current_robot_state["stop_initiated"] = False

                # switch gait based on speed
                mag = math.hypot(linear_x, linear_y)
                target_gait = RIPPLE_GAIT if (mag < SPEED_THRESHOLD_FOR_RIPPLE and abs(angular_z) < max_angular_speed * 0.3) else TRIPOD_GAIT
                if current_robot_state["active_gait"] != target_gait:
                    send_traveling_cmd(
                        gait_val=10 + target_gait,
                        period_val=current_robot_state["step_period"],
                        height_val=current_robot_state["step_height"]
                    )
                    current_robot_state["active_gait"] = target_gait

                # publish cmd_vel
                twist = Twist()
                twist.linear.x = linear_x
                twist.linear.y = linear_y
                twist.angular.z = angular_z
                cmd_vel_pub.publish(twist)

        elif current_operation_mode == POLICY_MODE:
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
            # AUTO logic here if needed
            pass

        rate.sleep()

    # cleanup
    if ser.is_open:
        ser.close()
    rospy.loginfo("[main] Exiting node")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("[__main__] Interrupted")
    except Exception as e:
        rospy.logerr(f"[__main__] Unhandled exception: {e}")

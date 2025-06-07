#!/usr/bin/env python3
# coding: utf-8

import rospy
import sys, select, termios, tty, math
from geometry_msgs.msg import Twist
from jethexa_controller_interfaces.msg import Traveling
from std_msgs.msg import String

class KeyboardGCSNode:
    def __init__(self):
        # 노드 초기화
        self.node_name = 'gcs_keyboard_to_cmd_vel_node'
        rospy.init_node(self.node_name)

        # 파라미터 로드
        self.max_linear_speed  = rospy.get_param("~max_linear_speed", 0.08)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.25)
        self.accel             = rospy.get_param("~accel", 0.005)

        # 상태 변수
        self.current_robot_state = {
            "active_gait": 1,  # TRIPOD_GAIT
            "step_height":  rospy.get_param("~default_step_height", 20.0),
            "step_period":  rospy.get_param("~default_step_period", 1.0),
            "is_commanding_movement": False,
            "stop_initiated": False
        }
        self.lin_x = 0.0
        self.lin_y = 0.0
        self.ang_z = 0.0

        # 퍼블리셔 설정
        self.cmd_vel_pub    = rospy.Publisher(
            '/jethexa/jethexa_controller/cmd_vel', Twist, queue_size=10)
        self.traveling_pub  = rospy.Publisher(
            '/jethexa/jethexa_controller/traveling', Traveling, queue_size=10)

        # 터미널 원복을 위한 설정 저장
        self.settings = termios.tcgetattr(sys.stdin)
        self.rate     = rospy.Rate(50)

        rospy.loginfo("Keyboard teleop ready. WASD: 이동, ←/→: 회전, Ctrl+C -> 종료")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key2 = sys.stdin.read(1)
                if key2 == '[':
                    key3 = sys.stdin.read(1)
                    key = '\x1b[' + key3
            return key
        return ''

    def send_traveling_cmd(self, gait_val, period=None, height=None,
                           stride=0.0, direction=0.0, rotation=0.0,
                           interrupt=True, steps=0):
        msg = Traveling()
        msg.gait            = int(gait_val)
        msg.stride          = float(stride)
        msg.height          = float(height if height is not None else self.current_robot_state["step_height"])
        msg.direction       = float(direction)
        msg.rotation        = float(rotation)
        msg.time            = float(period if period is not None else self.current_robot_state["step_period"])
        msg.steps           = int(steps)
        msg.interrupt       = bool(interrupt)
        msg.relative_height = False
        self.traveling_pub.publish(msg)

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                # Ctrl+C 처리
                if key == '\x03':  # Ctrl+C
                    rospy.loginfo("Keyboard teleop: Ctrl+C received, shutting down")
                    rospy.signal_shutdown("User requested shutdown")
                    break

                # 가속 로직
                # Linear X
                if key == 'w':
                    self.lin_x = min(self.lin_x + self.accel, self.max_linear_speed)
                elif key == 's':
                    self.lin_x = max(self.lin_x - self.accel, -self.max_linear_speed)
                else:
                    # 키가 놓이면 즉시 정지
                    self.lin_x = 0.0

                # Linear Y
                if key == 'a':
                    self.lin_y = min(self.lin_y + self.accel, self.max_linear_speed)
                elif key == 'd':
                    self.lin_y = max(self.lin_y - self.accel, -self.max_linear_speed)
                else:
                    self.lin_y = 0.0

                # Angular Z
                if key == '\x1b[D':
                    self.ang_z = min(self.ang_z + self.accel, self.max_angular_speed)
                elif key == '\x1b[C':
                    self.ang_z = max(self.ang_z - self.accel, -self.max_angular_speed)
                else:
                    self.ang_z = 0.0

                speed_mag = math.hypot(self.lin_x, self.lin_y)

                # 정지 상태
                if speed_mag < 1e-3 and abs(self.ang_z) < 1e-3:
                    if not self.current_robot_state["stop_initiated"]:
                        rospy.loginfo("Keyboard teleop: Stop")
                        self.send_traveling_cmd(gait_val=0)
                        self.current_robot_state["is_commanding_movement"] = False
                        self.current_robot_state["stop_initiated"]       = True

                else:
                    # 움직임 시작 시 한 번만 gait 설정
                    if not self.current_robot_state["is_commanding_movement"]:
                        rospy.loginfo("Keyboard teleop: Start moving")
                        gait = self.current_robot_state["active_gait"]
                        self.send_traveling_cmd(gait_val=10 + gait)
                        self.current_robot_state["stop_initiated"]        = False
                        self.current_robot_state["is_commanding_movement"] = True

                    # gait 전환
                    target_gait = 1  # TRIPOD
                    if speed_mag < 0.03 and abs(self.ang_z) < (self.max_angular_speed * 0.3):
                        target_gait = 2  # RIPPLE
                    if self.current_robot_state["active_gait"] != target_gait:
                        rospy.loginfo(f"Keyboard teleop: Change gait {self.current_robot_state['active_gait']} -> {target_gait}")
                        self.send_traveling_cmd(gait_val=10 + target_gait)
                        self.current_robot_state["active_gait"] = target_gait

                    # Twist 발행
                    twist = Twist()
                    twist.linear.x  = self.lin_x
                    twist.linear.y  = self.lin_y
                    twist.angular.z = self.ang_z
                    self.cmd_vel_pub.publish(twist)

                self.rate.sleep()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rospy.loginfo("Keyboard teleop: 종료")

if __name__ == "__main__":
    try:
        KeyboardGCSNode().run()
    except rospy.ROSInterruptException:
        pass


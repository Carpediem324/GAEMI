#!/usr/bin/env python3
# coding: utf-8

import threading
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from jethexa_controller_interfaces.msg import LegPosition
from geometry_msgs.msg import Point

class HelloServer:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('hello_server', anonymous=False)

        # 6번 다리 상대좌표 퍼블리셔
        self.pub = rospy.Publisher(
            '/jethexa/jethexa_controller/set_leg_relatively',
            LegPosition,
            queue_size=1
        )
        rospy.sleep(1.0)  # 퍼블리셔 연결 대기

        # 서비스 서버 생성
        self.running = False
        self.lock = threading.Lock()
        self.srv = rospy.Service(
            '/hello/set_running',
            SetBool,
            self.handle_set_running
        )
        rospy.loginfo("[HelloServer] Ready to greet on /hello/set_running")

    def handle_set_running(self, req):
        with self.lock:
            if req.data:
                if self.running:
                    return SetBoolResponse(
                        success=False,
                        message="이미 인사 중입니다."
                    )
                # 새 인사 스레드 시작
                self.running = True
                threading.Thread(target=self.greeting_sequence).start()
                return SetBoolResponse(
                    success=True,
                    message="인사를 시작합니다."
                )
            else:
                return SetBoolResponse(
                    success=False,
                    message="req.data=False 이므로 인사를 시작하지 않습니다."
                )

    def move_leg(self, leg_id, x, y, z, duration):
        msg = LegPosition()
        msg.leg_id = leg_id
        msg.position = Point(x=x, y=y, z=z)
        msg.duration = duration
        self.pub.publish(msg)
        rospy.loginfo(f"[HelloServer] move_leg → leg_id={leg_id}, pos=({x},{y},{z}), dur={duration}")

    def greeting_sequence(self):
        try:
            # 1) 원점시작
            self.move_leg(6, 0.0,  0.0,   0.0,   1.0)
            rospy.sleep(1.0)

            # 2) 손들기
            self.move_leg(6, 20.0, -20.0, 220.0, 1.0)
            rospy.sleep(1.0)

            # 3) 손흔들기(왼→오) 2회 반복
            for _ in range(2):
                # 왼쪽
                self.move_leg(6, 90.0,  60.0, 220.0, 1.0)
                rospy.sleep(1.0)
                # 오른쪽
                self.move_leg(6, 10.0, -40.0, 220.0, 1.0)
                rospy.sleep(1.0)

            # 4) 손내리기
            self.move_leg(6, 0.0,  0.0,   0.0,   1.0)
            rospy.sleep(1.0)

            rospy.loginfo("[HelloServer] 인사 동작 완료")
        finally:
            with self.lock:
                self.running = False

if __name__ == '__main__':
    server = HelloServer()
    rospy.spin()


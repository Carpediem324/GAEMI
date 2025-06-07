#!/usr/bin/env python3
# coding: utf-8

import rospy
import threading
from std_srvs.srv import SetBool, SetBoolResponse
from jethexa_controller_interfaces.msg import TransformEuler

class HeightServer:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('height_server', anonymous=False)

        # 퍼블리셔 설정 (pose_transform_euler 토픽 재사용)
        self.pub = rospy.Publisher(
            '/jethexa/jethexa_controller/pose_transform_euler',
            TransformEuler,
            queue_size=1
        )
        rospy.sleep(1.0)  # 퍼블리셔 연결 대기

        # 서비스 서버 생성
        self.lock = threading.Lock()
        self.srv = rospy.Service(
            '/height/set_running',
            SetBool,
            self.handle_set_running
        )
        rospy.loginfo("[HeightServer] Ready on /height/set_running")

    def handle_set_running(self, req):
        with self.lock:
            if req.data:
                self._lift_up()
                return SetBoolResponse(success=True, message="로봇을 1.0만큼 올렸습니다.")
            else:
                self._lower_down()
                return SetBoolResponse(success=True, message="로봇을 1.0만큼 내렸습니다.")

    def _lift_up(self):
        msg = TransformEuler()
        # translation 설정: z축 +1.0
        msg.translation.x = 0.0
        msg.translation.y = 0.0
        msg.translation.z = 2.5
        # rotation은 변화 없음
        msg.rotation.x = 0.0
        msg.rotation.y = 0.0
        msg.rotation.z = 0.0
        # 동작 지속시간
        msg.duration = 0.1
        self.pub.publish(msg)
        rospy.loginfo(f"[HeightServer] lift_up → translation.z={msg.translation.z:.2f}")

    def _lower_down(self):
        msg = TransformEuler()
        # translation 설정: z축 -1.0
        msg.translation.x = 0.0
        msg.translation.y = 0.0
        msg.translation.z = -2.5
        # rotation은 변화 없음
        msg.rotation.x = 0.0
        msg.rotation.y = 0.0
        msg.rotation.z = 0.0
        # 동작 지속시간
        msg.duration = 0.1
        self.pub.publish(msg)
        rospy.loginfo(f"[HeightServer] lower_down → translation.z={msg.translation.z:.2f}")

if __name__ == '__main__':
    server = HeightServer()
    rospy.spin()


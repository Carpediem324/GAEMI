#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from jethexa_controller_interfaces.msg import TransformEuler
import threading

class TiltingServer:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('tilting_server', anonymous=False)

        # 퍼블리셔 설정
        self.pub = rospy.Publisher(
            '/jethexa/jethexa_controller/pose_transform_euler',
            TransformEuler,
            queue_size=1
        )
        rospy.sleep(1.0)  # 연결 대기

        # 서비스 서버 생성
        self.lock = threading.Lock()
        self.srv = rospy.Service(
            '/tilting/set_running',
            SetBool,
            self.handle_set_running
        )
        rospy.loginfo("[TiltingServer] Ready on /tilting/set_running")

    def handle_set_running(self, req):
        with self.lock:
            if req.data:
                self._tilt_up()
                return SetBoolResponse(True, "틸트 업 동작을 발행했습니다.")
            else:
                self._tilt_down()
                return SetBoolResponse(True, "틸트 다운 동작을 발행했습니다.")

    def _tilt_up(self):
        msg = TransformEuler()
        # translation 필드 설정
        msg.translation.x = 0.0
        msg.translation.y = 0.0
        msg.translation.z = 0.0
        # rotation 필드 설정 (예: pitch ↑)
        msg.rotation.x = 0.0
        msg.rotation.y = 0.05    # 위로 기울이려는 값
        msg.rotation.z = 0.0
        # 지속시간
        msg.duration = 1.0
        self.pub.publish(msg)
        rospy.loginfo(f"[TiltingServer] tilt_up → rotation.y={msg.rotation.y:.2f}")

    def _tilt_down(self):
        msg = TransformEuler()
        # translation 필드 설정
        msg.translation.x = 0.0
        msg.translation.y = 0.0
        msg.translation.z = 0.0
        # rotation 필드 설정 (예: pitch ↓)
        msg.rotation.x = 0.0
        msg.rotation.y = -0.05   # 아래로 기울이려는 값
        msg.rotation.z = 0.0
        # 지속시간
        msg.duration = 1.0
        self.pub.publish(msg)
        rospy.loginfo(f"[TiltingServer] tilt_down → rotation.y={msg.rotation.y:.2f}")

if __name__ == '__main__':
    server = TiltingServer()
    rospy.spin()


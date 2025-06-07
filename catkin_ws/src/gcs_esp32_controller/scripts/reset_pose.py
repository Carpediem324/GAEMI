#!/usr/bin/env python3
# coding: utf-8

import rospy
import threading
from std_srvs.srv import Trigger, TriggerResponse
from jethexa_controller_interfaces.msg import Traveling

class ResetPoseServer:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('reset_pose_server', anonymous=False)

        # 퍼블리셔 설정
        self.pub = rospy.Publisher(
            '/jethexa/jethexa_controller/traveling',
            Traveling,
            queue_size=1
        )
        rospy.sleep(1.0)  # 연결 대기

        # 서비스 서버 생성
        self.lock = threading.Lock()
        self.srv = rospy.Service(
            '/reset_pose',
            Trigger,
            self.handle_reset_pose
        )
        rospy.loginfo("[ResetPoseServer] Ready on /reset_pose")

    def handle_reset_pose(self, req):
        with self.lock:
            self._publish_reset()
            return TriggerResponse(
                success=True,
                message="로봇 자세를 초기자세로 변경했습니다."
            )

    def _publish_reset(self):
        msg = Traveling()
        # echo에서 확인된 초기값 설정
        msg.gait = -2
        msg.stride = 30.0
        msg.height = 15.0
        msg.direction = 0.0
        msg.rotation = 0.0
        msg.time = 1.0
        msg.steps = 0
        msg.relative_height = False
        msg.interrupt = True
        self.pub.publish(msg)
        rospy.loginfo("[ResetPoseServer] Published reset pose → interrupt=True, height=15.0")

if __name__ == '__main__':
    server = ResetPoseServer()
    rospy.spin()


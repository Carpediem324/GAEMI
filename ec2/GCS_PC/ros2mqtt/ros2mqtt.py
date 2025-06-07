#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Noetic → MQTT 브리지
© 2025 Shin Hyeonhak
"""

import os
import json
import time
from typing import Any, List, Dict

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
from dotenv import load_dotenv


class RosMqttBridge:
    """ROS1 토픽을 MQTT(JSON)으로 변환·중계"""

    def __init__(self) -> None:
        # 1) .env 로드
        load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))
        self.mqtt_host: str = os.getenv("host", "localhost")
        self.mqtt_port: int = int(os.getenv("port", "1883"))
        self.mqtt_user: str = os.getenv("user", "")
        self.mqtt_pass: str = os.getenv("pass", "")

        # 2) MQTT 클라이언트
        self.client = mqtt.Client(client_id="ros_mqtt_bridge_noetic", protocol=mqtt.MQTTv5)
        if self.mqtt_user:
            self.client.username_pw_set(self.mqtt_user, self.mqtt_pass)
        self.client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
        self.client.loop_start()
        
        # 3) ROS 연결 상태 변수
        self.is_ros_connected = False
        
        # 4) ROS 초기화 시도
        self._init_ros()
        
        # 5) ROS 연결 상태 타이머 시작 (1초마다 체크 및 상태 발행)
        self._ros_check_timer = rospy.Timer(rospy.Duration(1), self._check_ros_connection)
        
        rospy.on_shutdown(self._cleanup)
        rospy.spin()

    def _init_ros(self) -> None:
        """ROS 초기화 및 연결 시도"""
        while True:
            try:
                # ROS 노드 초기화 시도
                rospy.init_node("ros_mqtt_bridge", anonymous=False)
                self.is_ros_connected = True
                self._publish_ros_status(True)
                self._init_subscribers()
                rospy.loginfo("✅ ROS ↔ MQTT 브리지가 실행되었습니다.")
                return  # 성공 시 함수 종료
            except Exception as e:
                print(f"roscore 연결 실패: {e}. 재연결 시도 중...")
                self._publish_ros_status(False)
                time.sleep(5)
                # while 루프를 통해 재시도 (재귀 호출 제거)
    
    def _check_ros_connection(self, _) -> None:
        """ROS 연결 상태 주기적 확인 및 상태 변경 시 발행"""
        previous_status = self.is_ros_connected  # 이전 상태 저장
        
        try:
            # rospy.get_master() 호출로 연결 확인
            rospy.get_master().getSystemState()
            current_status = True
            if not previous_status:
                print("roscore 연결이 복구되었습니다.")
                # 재연결 시 구독자 다시 등록 (상태 복구 시 1번만 실행)
                self._init_subscribers()
        except Exception as e:
            current_status = False
            if previous_status:
                print(f"roscore 연결이 끊겼습니다: {e}")
        
        # 상태가 변경되었을 때만 발행
        if current_status != previous_status:
            self._publish_ros_status(current_status)
            self.is_ros_connected = current_status  # 상태 업데이트
    
    def _publish_ros_status(self, status: bool) -> None:
        """ROS 연결 상태를 MQTT로 발행"""
        self._publish_mqtt("ros_status", status)

    # --------------------------------------------------
    # 구독자 등록
    # --------------------------------------------------
    def _init_subscribers(self) -> None:
        rospy.Subscriber(
            "/jethexa/voltage",
            Float32,
            lambda msg: self._publish_mqtt("voltage", msg.data),
            queue_size=10,
        )

        rospy.Subscriber(
            "/air_sensor/dust",
            Float32MultiArray,
            lambda msg: self._publish_mqtt("dust", self._array_to_list(msg)),
            queue_size=10,
        )

        rospy.Subscriber(
            "/air_sensor/gas",
            Bool,
            lambda msg: self._publish_mqtt("gas", bool(msg.data)),
            queue_size=10,
        )

        rospy.Subscriber(
            "/jethexa/joint_states",
            JointState,
            lambda msg: self._publish_mqtt("joint_states", self._jointstate_to_dict(msg)),
            queue_size=10,
        )

        rospy.Subscriber(
            "/jethexa/jethexa_controller/cmd_vel",
            Twist,
            lambda msg: self._publish_mqtt("cmd_vel", self._twist_to_dict(msg)),
            queue_size=10,
        )

    # --------------------------------------------------
    # ROS → Python 기본 자료형 변환
    # --------------------------------------------------
    @staticmethod
    def _array_to_list(msg: Float32MultiArray) -> List[float]:
        return list(msg.data)

    @staticmethod
    def _jointstate_to_dict(msg: JointState) -> Dict[str, Any]:
        return {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }

    @staticmethod
    def _twist_to_dict(msg: Twist) -> Dict[str, Any]:
        return {
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z
            }
        }

    # --------------------------------------------------
    # MQTT 발행
    # --------------------------------------------------
    def _publish_mqtt(self, topic: str, data: Any) -> None:
        """
        모든 payload를 {"topic": <토픽명>, "data": <데이터>} 형식의
        JSON 문자열로 변환 후 발행(QoS 0, retain False)
        """
        payload_obj = {"topic": topic, "data": data}
        try:
            json_payload = json.dumps(payload_obj, ensure_ascii=False)
            result = self.client.publish(topic, json_payload, qos=0, retain=False)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                rospy.logwarn(f"[MQTT] {topic} 발행 실패(rc={result.rc})")
        except Exception as e:
            rospy.logerr(f"[MQTT] {topic} 발행 중 예외: {e}")

    # --------------------------------------------------
    # 종료 정리
    # --------------------------------------------------
    def _cleanup(self) -> None:
        """ROS 종료 시 MQTT 루프·연결 정리"""
        try:
            self._publish_ros_status(False) # ROS 상태를 False로 발행
            if hasattr(self, '_ros_check_timer'):
                self._ros_check_timer.shutdown()
            self.client.loop_stop()
            self.client.disconnect()
            rospy.loginfo("🛑 MQTT 연결 종료")
        except Exception as e:
            rospy.logwarn(f"[MQTT] 종료 중 예외 무시: {e}")


# ------------------------------------------------------
# 메인
# ------------------------------------------------------
if __name__ == "__main__":
    try:
        RosMqttBridge()
    except rospy.ROSInterruptException:
        pass


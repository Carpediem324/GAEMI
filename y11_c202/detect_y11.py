#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 (Noetic) Person Detector Node (YOLO11 포팅 버전)
– roscore 연결 대기
– 경고 억제
– GPU(cuda) 가속, half precision, fuse
– Frame-drop 로직: RGB/IR 각각 독립 적용
– 박스 색상 변경: 연두색
© 2025 Shin Hyeonhak
"""

import warnings
warnings.filterwarnings(
    "ignore",
    message=".*torch\\.cuda\\.amp\\.autocast.*",
    category=FutureWarning
)

import os
import time
import torch
import cv2
import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# 로그 억제를 위한 설정
import logging
logging.getLogger('ultralytics').setLevel(logging.WARNING)

# --- 수정된 부분: YOLO11 API import ---
from ultralytics import YOLO

class PersonDetectorNode:
    def __init__(self):
        # --- roscore 연결 대기 ---
        self.ros_master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        self.ros_connected = False
        while not rosgraph.is_master_online(self.ros_master_uri):
            print(f"[PersonDetectorNode] roscore 연결 실패: {self.ros_master_uri}\n 대기 중... (재시도)")
            time.sleep(1)
        self.ros_connected = True
        rospy.loginfo(f"[PersonDetectorNode] roscore 연결 성공: {self.ros_master_uri}")

        # ROS 노드 초기화
        rospy.init_node('person_detector_node')
        self.bridge = CvBridge()

        # 1) Device & CUDNN 튜닝
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        torch.backends.cudnn.benchmark = True
        rospy.loginfo(f"[YOLO] 사용 디바이스: {self.device}")

        # 2) YOLO11 모델 로드 & 최적화
        rospy.loginfo("[YOLO] YOLO11 모델 로드 중...")
        self.model = YOLO("yolo11n.pt")
        self.model.to(self.device)
        if self.device.type == 'cuda':
            self.model.model.half()
            if hasattr(self.model.model, 'fuse') and callable(self.model.model.fuse):
                self.model.model.fuse()
        self.model.model.eval()
        self.classes = [0]  # person 클래스

        # 3) Frame-drop 설정
        self.max_fps = 15.0
        self.min_interval = 1.0 / self.max_fps
        self.last_color_time = time.time() - self.min_interval
        self.last_infra_time = time.time() - self.min_interval

        # 4) 토픽 구독/발행
        sub_kwargs = dict(queue_size=1, buff_size=2**20)
        rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.cb_color, **sub_kwargs)
        rospy.Subscriber('/camera/infra1/image_rect_raw/compressed', CompressedImage, self.cb_infra, **sub_kwargs)
        self.pub_color = rospy.Publisher('/yolo/color/compressed', CompressedImage, queue_size=1)
        self.pub_infra = rospy.Publisher('/yolo/infra1/compressed', CompressedImage, queue_size=1)

        # ROS 연결 상태 주기적 확인 타이머
        rospy.Timer(rospy.Duration(1), self.check_ros_connection)

        rospy.loginfo("[YOLO] Node started")
        rospy.spin()

    def check_ros_connection(self, event=None):
        current_status = rosgraph.is_master_online(self.ros_master_uri)
        if current_status != self.ros_connected:
            if not current_status:
                rospy.logwarn("[PersonDetectorNode] ROS Master 연결 끊김. 재연결을 시도합니다...")
                while not rosgraph.is_master_online(self.ros_master_uri):
                    if rospy.is_shutdown():
                        rospy.logwarn("[PersonDetectorNode] 노드 종료로 인해 ROS Master 재연결 시도 중단.")
                        return
                    rospy.logwarn(f"[PersonDetectorNode] ROS Master 재연결 시도 중... ({self.ros_master_uri})")
                    time.sleep(1)
                rospy.loginfo("[PersonDetectorNode] ROS Master 재연결 성공.")
            self.ros_connected = current_status

    def yolo_process(self, img: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        with torch.no_grad():
            results = self.model.predict(
                source=rgb,
                device=self.device,
                classes=self.classes,
                imgsz=640,
                conf=0.25,
                iou=0.45,
                half=self.device.type == 'cuda',
                verbose=False,
                stream=False
            )
        boxes = results[0].boxes
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        for (x1, y1, x2, y2), conf in zip(xyxy, confs):
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(img, f'Person {conf:.2f}', (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
        return img

    def cb_color(self, msg: CompressedImage):
        now = time.time()
        if now - self.last_color_time < self.min_interval: return
        self.last_color_time = now
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            out = self.yolo_process(img)
            _, buf = cv2.imencode('.jpg', out)
            cm = CompressedImage(); cm.header=msg.header; cm.format='jpeg'; cm.data=buf.tobytes()
            self.pub_color.publish(cm)
        except Exception as e:
            rospy.logerr(f"[YOLO RGB 오류] {e}")

    def cb_infra(self, msg: CompressedImage):
        now = time.time()
        if now - self.last_infra_time < self.min_interval: return
        self.last_infra_time = now
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            out = self.yolo_process(img)
            _, buf = cv2.imencode('.jpg', out)
            cm = CompressedImage(); cm.header=msg.header; cm.format='jpeg'; cm.data=buf.tobytes()
            self.pub_infra.publish(cm)
        except Exception as e:
            rospy.logerr(f"[YOLO IR 오류] {e}")

if __name__ == '__main__':
    try:
        PersonDetectorNode()
    except rospy.ROSInterruptException:
        pass


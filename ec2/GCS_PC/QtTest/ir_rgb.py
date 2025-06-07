#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 (Noetic) 이미지 시각화 → PyQt5 뷰어
RGB 컬러(/camera/color/image_raw/compressed) + IR(/camera/infra1/image_rect_raw/compressed) + Depth(/camera/depth/image_rect_raw/compressed) 동시 표시
기본적으로 압축 이미지를 사용하여 대역폭과 처리 속도를 향상시킵니다.
"""

import sys
from typing import Optional, Union, Dict, Any
from collections import deque

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import threading

from PyQt5.QtWidgets import (
    QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QSizePolicy, 
    QPushButton, QComboBox, QSlider, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, QMutex
from PyQt5.QtGui import QImage, QPixmap


class ImageViewer:
    """ROS 이미지(압축/비압축) 토픽을 QLabel에 실시간 표시."""

    def __init__(self, label: QLabel, topic: str, is_depth: bool = False, 
                 use_compressed: bool = True):
        self._label: QLabel = label
        self._bridge: CvBridge = CvBridge()
        self._is_depth = is_depth  # Depth 이미지인지 여부
        self._use_compressed = use_compressed  # 압축 이미지 사용 여부
        self._topic_base = topic  # 기본 토픽 경로
        self._last_frame_time = rospy.Time.now()  # 마지막 프레임 수신 시간
        self._frame_count = 0  # 수신된 프레임 수
        self._display_fps = 0  # 표시 FPS
        self._last_image = None  # 마지막 이미지 저장
        self._displayed_image = None  # 현재 표시 중인 이미지
        self._decoder_thread_count = 2  # OpenCV 디코더 스레드 수
        self._image_mutex = QMutex()  # 이미지 처리 동기화를 위한 뮤텍스
        
        # Depth 이미지 안정화를 위한 변수
        self._depth_min_max_history = deque(maxlen=5)  # 최근 5프레임의 min/max 값 저장
        self._depth_range = (0.1, 8.0)  # 기본 깊이 범위 (미터)
        
        # 컬러맵 선택 (JET 대신 RAINBOW 사용 - 빨간색 덜 강조)
        self._colormap = cv2.COLORMAP_RAINBOW
        
        # OpenCV 병렬 처리 최적화
        cv2.setNumThreads(8)  # 병렬 처리 스레드 수 증가
        
        # FPS 측정용 타이머 - 표시는 별도 타이머로 변경
        self._stats_timer = QTimer()
        self._stats_timer.timeout.connect(self._update_stats)
        self._stats_timer.start(2000)  # 2초마다 통계 업데이트 (더 긴 주기로 변경)
        
        # 통계 텍스트
        self._stats_text = ""
        
        # 구독자 설정
        self._setup_subscriber()
    
    def _setup_subscriber(self):
        """토픽 구독 설정"""
        if self._use_compressed:
            topic = f"{self._topic_base}/compressed"
            msg_type = CompressedImage
        else:
            topic = self._topic_base
            msg_type = Image
            
        rospy.loginfo(f"[ImageViewer] 토픽 구독 시작: {topic}")
        
        # 기존 구독자 종료
        if hasattr(self, '_sub') and self._sub is not None:
            self._sub.unregister()
            
        # 새 구독자 생성
        self._sub = rospy.Subscriber(
            topic,
            msg_type,
            self._image_callback,
            queue_size=2,  # 압축 이미지에 최적화된 큐 크기
            buff_size=2 ** 24
        )
    
    def toggle_compressed(self, use_compressed: bool):
        """압축/비압축 토픽 전환"""
        if self._use_compressed != use_compressed:
            self._use_compressed = use_compressed
            self._setup_subscriber()
    
    def _update_stats(self):
        """FPS 측정 및 통계 업데이트 - 표시는 하지 않고 텍스트만 업데이트"""
        now = rospy.Time.now()
        duration = (now - self._last_frame_time).to_sec()
        if duration > 0:
            self._display_fps = self._frame_count / duration
            self._frame_count = 0
            self._last_frame_time = now
            
            # 통계 텍스트 업데이트만 수행
            topic_type = "압축" if self._use_compressed else "비압축"
            if self._last_image is not None:
                img_shape = self._last_image.shape
                self._stats_text = f"FPS: {self._display_fps:.1f} | {img_shape[1]}x{img_shape[0]} | {topic_type}"
            
            # 이미지는 건드리지 않음

    # --------------------------------------------------------------------- #
    #                           ROS 콜백                                   #
    # --------------------------------------------------------------------- #
    def _image_callback(self, msg: Union[Image, CompressedImage]) -> None:
        try:
            # 프레임 카운트 증가
            self._frame_count += 1
            
            # 이미지 디코딩 및 변환
            cv_image = self._decode_image(msg)
            
            if cv_image is None:
                rospy.logwarn(f"이미지 디코딩 실패: {self._topic_base}")
                return
                
            # 이미지 뮤텍스 잠금
            self._image_mutex.lock()
            try:
                # 이미지 저장
                self._last_image = cv_image.copy()
                
                # 이미지 타입 별 처리
                if self._is_depth:
                    processed_image = self._process_depth_image(cv_image)
                else:
                    processed_image = self._process_rgb_ir_image(cv_image)
                
                # 처리된 이미지가 있으면 표시
                if processed_image is not None:
                    # 통계 텍스트가 있으면 오버레이 추가
                    if self._stats_text:
                        # 화면 하단에 작게 통계 텍스트 추가
                        h, w = processed_image.shape[:2]
                        text_start_x = 10
                        text_start_y = h - 10  # 하단에 배치
                        cv2.putText(
                            processed_image, 
                            self._stats_text, 
                            (text_start_x, text_start_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
                        )
                    
                    # 이미지 표시
                    self._displayed_image = processed_image
                    self._display_image(processed_image)
            finally:
                # 이미지 뮤텍스 해제
                self._image_mutex.unlock()
                
        except CvBridgeError as e:
            rospy.logerr(f"[cv_bridge 오류] {e}")
        except Exception as e:
            rospy.logerr(f"[예외] {type(e).__name__}: {e}")
    
    def _decode_image(self, msg: Union[Image, CompressedImage]) -> Optional[np.ndarray]:
        """이미지 메시지 디코딩"""
        try:
            if isinstance(msg, CompressedImage):
                np_arr = np.frombuffer(msg.data, np.uint8)
                
                if self._is_depth:
                    # Depth 압축 이미지 처리 
                    flags = cv2.IMREAD_ANYDEPTH | cv2.IMREAD_IGNORE_ORIENTATION
                    return cv2.imdecode(np_arr, flags)
                else:
                    # RGB/IR 이미지 디코딩
                    flags = cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION
                    return cv2.imdecode(np_arr, flags)
            else:
                # 비압축 이미지 처리
                if self._is_depth:
                    return self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"[이미지 디코딩 오류] {e}")
            return None
    
    def _process_depth_image(self, cv_image: np.ndarray) -> Optional[np.ndarray]:
        """뎁스 이미지 처리"""
        try:
            # 16비트 정수형 타입 확인
            if cv_image.dtype == np.uint16 or cv_image.dtype == np.int16:
                # 고정 범위 사용 (밀리미터 단위)
                min_depth_mm = int(self._depth_range[0] * 1000)  # 0.1m -> 100mm
                max_depth_mm = int(self._depth_range[1] * 1000)  # 8.0m -> 8000mm
                
                # 최대/최소 값이 실제 데이터에 맞는지 확인 (선택적으로 조정)
                valid_pixels = cv_image[cv_image > 0]
                if len(valid_pixels) > 100:  # 유효한 픽셀이 충분히 있는 경우
                    # 실제 최소값을 min_depth_mm과 비교하여 더 큰 값 사용
                    actual_min = np.percentile(valid_pixels, 1)  # 하위 1% 값
                    min_depth_mm = max(min_depth_mm, int(actual_min * 0.95))
                    
                    # 실제 최대값을 max_depth_mm과 비교하여 더 작은 값 사용
                    actual_max = np.percentile(valid_pixels, 99)  # 상위 99% 값
                    max_depth_mm = min(max_depth_mm, int(actual_max * 1.05))
                
                # 직접 고정 범위로 스케일링하여 깜빡임 감소
                depth_8bit = np.zeros(cv_image.shape, dtype=np.uint8)
                
                # 유효한 깊이 값 (0보다 크고 max_depth_mm보다 작은 값) 마스크
                valid_mask = (cv_image > 0) & (cv_image < max_depth_mm)
                
                if np.any(valid_mask):
                    # 이미지 스케일링 (0-255 범위로)
                    ratio = 255.0 / (max_depth_mm - min_depth_mm)
                    depth_8bit[valid_mask] = np.clip(
                        (cv_image[valid_mask] - min_depth_mm) * ratio, 
                        0, 255
                    ).astype(np.uint8)
                    
                    # 노이즈 감소를 위한 중간값 필터 - 옵션
                    if np.mean(depth_8bit) > 10:  # 평균값이 일정 이상인 경우에만 필터 적용
                        depth_8bit = cv2.medianBlur(depth_8bit, 5)
                
                # 컬러맵 적용 (RAINBOW는 색상 분포가 더 균일)
                depth_colormap = cv2.applyColorMap(depth_8bit, self._colormap)
                
                # 무효 데이터는 검은색
                depth_colormap[cv_image == 0] = [0, 0, 0]
                
                # 깊이 범위 정보 오버레이
                min_str = f"{min_depth_mm/1000:.1f}m"
                max_str = f"{max_depth_mm/1000:.1f}m"
                cv2.putText(
                    depth_colormap,
                    f"범위: {min_str} - {max_str}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
                )
                
                return depth_colormap
            elif cv_image.dtype == np.uint8:
                # 이미 8비트인 경우 바로 컬러맵 적용
                return cv2.applyColorMap(cv_image, self._colormap)
            else:
                # 지원되지 않는 타입
                self._show_message(f"지원되지 않는 깊이 이미지 형식: {cv_image.dtype}")
                return None
        except Exception as e:
            rospy.logerr(f"[깊이 이미지 처리 오류] {type(e).__name__}: {e}")
            # 기존 표시된 이미지 유지
            return self._displayed_image if self._displayed_image is not None else None
    
    def _process_rgb_ir_image(self, cv_image: np.ndarray) -> Optional[np.ndarray]:
        """RGB/IR 이미지 처리"""
        try:
            # IR 이미지 처리
            if 'infra' in self._topic_base and len(cv_image.shape) >= 2:
                # 이미지가 3채널이면 그레이스케일로 변환
                if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                else:
                    gray = cv_image  # 이미 그레이스케일
                    
                # 히스토그램 균일화로 대비 향상
                gray = cv2.equalizeHist(gray)
                # 노이즈 감소
                gray = cv2.GaussianBlur(gray, (5, 5), 0)
                # 다시 컬러로 변환 (시각화 목적)
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                # RGB 이미지는 그대로 반환
                return cv_image
        except Exception as e:
            rospy.logerr(f"[RGB/IR 이미지 처리 오류] {type(e).__name__}: {e}")
            return cv_image  # 오류 발생 시 원본 반환
    
    def _show_message(self, message: str):
        """이미지 대신 텍스트 메시지 표시"""
        # 검은 배경의 이미지 생성
        h, w = 480, 640
        img = np.zeros((h, w, 3), dtype=np.uint8)
        
        # 텍스트 추가
        font = cv2.FONT_HERSHEY_SIMPLEX
        # 텍스트 크기 계산
        text_size = cv2.getTextSize(message, font, 1, 2)[0]
        
        # 텍스트 위치 계산 (중앙)
        x = (w - text_size[0]) // 2
        y = (h + text_size[1]) // 2
        
        # 텍스트 그리기
        cv2.putText(img, message, (x, y), font, 1, (255, 255, 255), 2)
        
        # 이미지 표시
        self._display_image(img)
    
    def _display_image(self, cv_image: np.ndarray):
        """OpenCV 이미지를 QImage로 변환하여 표시"""
        try:
            # BGR 이미지 확인
            if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                height, width, channel = rgb.shape
                bytes_per_line = channel * width
                qt_img = QImage(
                    rgb.data, width, height,
                    bytes_per_line, QImage.Format_RGB888
                )
            # 그레이스케일 이미지
            elif len(cv_image.shape) == 2:
                height, width = cv_image.shape
                bytes_per_line = width
                qt_img = QImage(
                    cv_image.data, width, height,
                    bytes_per_line, QImage.Format_Grayscale8
                )
            else:
                rospy.logwarn(f"지원되지 않는 이미지 형식: {cv_image.shape}")
                return
                
            pixmap = QPixmap.fromImage(qt_img)
            
            self._label.setPixmap(
                pixmap.scaled(
                    self._label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
            )
        except Exception as e:
            rospy.logerr(f"[이미지 변환 오류] {e}")
    
    def _add_text_overlay(self, text: str):
        """이미지에 텍스트 오버레이 추가 - 더 이상 사용하지 않음"""
        pass  # 이 함수는 더 이상 사용하지 않지만 호환성을 위해 유지


def main() -> None:
    rospy.init_node("qt_image_viewer", anonymous=False)

    # ───────── Qt 애플리케이션 준비 ───────── #
    app = QApplication(sys.argv)

    # QLabel 세 개 생성
    rgb_label = QLabel("RGB 이미지 대기 중…")
    ir_label = QLabel("IR 이미지 대기 중…")
    depth_label = QLabel("Depth 이미지 대기 중…")

    # 동일 크기 유지 + 빈 공간 채우기
    for lbl in (rgb_label, ir_label, depth_label):
        lbl.setFixedSize(640, 480)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    # 압축/비압축 전환 버튼
    compress_toggle = QComboBox()
    compress_toggle.addItems(["비압축 이미지", "압축 이미지"])
    compress_toggle.setCurrentIndex(1)  # 기본값: 압축(인덱스 1)

    # 윈도우·레이아웃 - 상단에 RGB/IR, 하단에 Depth
    window = QWidget()
    main_layout = QVBoxLayout(window)
    
    # 상단 컨트롤 영역
    control_layout = QHBoxLayout()
    control_layout.addWidget(compress_toggle)
    main_layout.addLayout(control_layout)
    
    # 이미지 레이아웃
    img_layout = QGridLayout()
    img_layout.addWidget(rgb_label, 0, 0)
    img_layout.addWidget(ir_label, 0, 1)
    img_layout.addWidget(depth_label, 1, 0, 1, 2)  # 하단 전체 영역
    main_layout.addLayout(img_layout)
    
    window.setWindowTitle("RGB / IR / Depth 이미지 뷰어 (압축)")
    window.show()

    # ────── 이미지 뷰어 인스턴스(세 스트림) ────── #
    use_compressed = compress_toggle.currentIndex() == 1  # 기본값: 압축
    
    # 뷰어 생성
    rgb_viewer = ImageViewer(
        rgb_label,
        topic="/camera/color/image_raw",
        use_compressed=use_compressed
    )
    
    ir_viewer = ImageViewer(
        ir_label,
        topic="/camera/infra1/image_rect_raw",
        use_compressed=use_compressed
    )
    
    depth_viewer = ImageViewer(
        depth_label,
        topic="/camera/depth/image_rect_raw",
        is_depth=True,
        use_compressed=use_compressed
    )
    
    # 압축/비압축 전환 기능 연결
    def toggle_compression(index):
        use_compressed = index == 1
        rgb_viewer.toggle_compressed(use_compressed)
        ir_viewer.toggle_compressed(use_compressed)
        depth_viewer.toggle_compressed(use_compressed)
        
    compress_toggle.currentIndexChanged.connect(toggle_compression)

    # Qt 이벤트 루프 실행
    exit_code = app.exec_()

    # 종료 처리
    rospy.signal_shutdown("Qt 종료")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()


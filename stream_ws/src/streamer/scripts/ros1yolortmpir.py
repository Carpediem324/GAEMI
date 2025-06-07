#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import subprocess
import threading
import time
import os
import psutil

class YOLOIRRtmpStreamer:
    def __init__(self):
        #변수 초기화
        self.ffmpeg = None
        self.ffmpeg_lock = threading.Lock()
        self.restart_count = 0
        self.max_restarts = 5
        self.frame_count = 0
        self._current_resolution = None
        self.debug_mode = True

        self.last_image_time = time.time()
        self.timeout_sec = rospy.get_param('~timeout_sec', 10)  # 수신 없을 시 종료 대기 시간

        # ROS 노드 초기화
        rospy.init_node('yolo_IR_rtmp_streamer', anonymous=True)

        self.rtmp_url = rospy.get_param('~rtmp_url', 'rtmp://gaemibot.duckdns.org/LiveApp/ir')
        self.frame_rate = rospy.get_param('~frame_rate', 15)
        self.preset = rospy.get_param('~preset', 'ultrafast')
        self.debug_mode = rospy.get_param('~debug_mode', False)

        image_topic = rospy.get_param('~image_topic', '/yolo/infra1/compressed')
        # 토픽 구독
        self.subscription = rospy.Subscriber(
            image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=10
        )

        
        self.ffmpeg = self.start_ffmpeg()
        rospy.loginfo("Compressed RTMP 스트리머 시작됨")

        # 노드 종료 시 정리 작업 등록
        rospy.on_shutdown(self.shutdown)

    def start_timer(self):
        self.memory_check_timer = rospy.Timer(rospy.Duration(60), self._check_memory_usage)
        self.timeout_check_timer = rospy.Timer(rospy.Duration(1), self._check_timeout)
    
    def _check_memory_usage(self, event):
        # 메모리 사용량 확인
        process = psutil.Process(os.getpid())
        memory_usage = process.memory_info().rss / 1024 / 1024 # MB 단위로 변환
        rospy.loginfo(f"현재 메모리 사용량 : {memory_usage : 0.2f} MB")


    def ensure_ffmpeg_running(self, width, height):
        with self.ffmpeg_lock:
            if self.ffmpeg is None:
                rospy.loginfo(f"FFmpeg 시작: 초기화")
                self._current_resolution = (width, height)
                self.ffmpeg = self.start_ffmpeg(width, height)
                return self.ffmpeg is not None
            elif self.ffmpeg.poll() is not None:
                rospy.loginfo(f"FFmpeg 재시작: 프로세스 종료됨")
                self.ffmpeg = self.start_ffmpeg(self._current_resolution[0], self._current_resolution[1])
                return self.ffmpeg is not None
            return True

    
    def start_ffmpeg(self, width=424, height=240):
        with self.ffmpeg_lock:
            if self.ffmpeg is not None:
                self.ffmpeg.terminate()
                self.ffmpeg.wait()

            if self.restart_count > self.max_restarts:
                rospy.logerr("최대 재시작 횟수 초과")
                return None
            
            self.restart_count += 1

        cmd=[
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            '-r', str(self.frame_rate),
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', self.preset,
            '-tune', 'zerolatency',
            '-g', '30',             # GOP 크기 설정 (키프레임 간격)
            '-keyint_min', '30',    # 최소 키프레임 간격
            '-b:v', '800k',           # 비트레이트 설정
            '-maxrate', '800k',       # 최대 비트레이트
            '-bufsize', '2M',       # 버퍼 크기
            '-flags', '+global_header', # 글로벌 헤더 추가
            '-bsf:v', 'dump_extra', # 비트스트림 필터
            '-f', 'flv',
            '-flvflags', 'no_duration_filesize',
            self.rtmp_url
        ]

        rospy.loginfo(f"FFmpeg 시작: 해상도 {width}x{height}")
        self.ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        if self.ffmpeg:
            # ROS1이라 timer 사용
            timer = threading.Timer(300, self._reset_restart_count)
            # 데몬 스레드로 설정해 메인과 함께 종료되게 설정
            timer.daemon = True
            timer.start()

            stderr_thread = threading.Thread(target=self._monitor_ffmpeg_stderr)
            stderr_thread.daemon = True
            stderr_thread.start()

        return self.ffmpeg
    
    
    def _monitor_ffmpeg_stderr(self):
        # FFmpeg stderr 출력 모니터링
        while self.ffmpeg and self.ffmpeg.poll() is None:
            line = self.ffmpeg.stderr.readline()
            if line:
                line_text = line.decode(errors='ignore').strip()
                # 모든 FFmpeg 출력을 INFO 레벨로 로깅
                rospy.loginfo(f"FFmpeg: {line_text}")


    def _reset_restart_count(self):
        with self.ffmpeg_lock:
            if self.ffmpeg and self.ffmpeg.poll() is None:
                self.restart_count = 0
                rospy.loginfo("FFmpeg 안정적으로 실행 중: 재시작 카운터 리셋")

    def image_callback(self, msg):
        if self.frame_count % 100 == 0:
            rospy.loginfo(f"이미지 처리 중: {self.frame_count}번째 프레임")
        self.frame_count += 1

        self.last_image_time = time.time()  # 프레임 수신 시간 업데이트

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            rospy.logwarn("프레임 디코딩 실패")
            return
        
        height, width = frame.shape[:2]

        if not self.ensure_ffmpeg_running(width, height):
            rospy.logerr("FFmpeg 시작 실패")
            return

        if self.debug_mode:
            #cv2.imshow("RealSense YOLO Frame", frame)
            cv2.waitKey(1)

        try:
            with self.ffmpeg_lock:
                if self.ffmpeg:
                    self.ffmpeg.stdin.write(frame.tobytes())

        except BrokenPipeError:
            rospy.logerr("파이프 연결이 끊어졌습니다. FFmpeg 프로세스 재시작 시도")
            with self.ffmpeg_lock:
                # 다음 반복에서 재시작
                self.ffmpeg = None
            
        # OS 관련
        except OSError as e:
            rospy.logerr(f"OS 오류 발생: {e}")

        # 기타 예외
        except Exception as e:
            rospy.logerr(f"FFmpeg 오류: {e}")
            with self.ffmpeg_lock:
                if self.ffmpeg:
                    stderr_output = self.ffmpeg.stderr.read().decode(errors='ignore')
                    rospy.logerr(f" FFmpeg stderr: {stderr_output}")
        

    def _check_timeout(self, event):
        if time.time() - self.last_image_time > self.timeout_sec:
            rospy.logwarn(f"{self.timeout_sec}초 동안 이미지 수신이 없어 노드를 종료합니다.")
            rospy.signal_shutdown("이미지 수신 중단")
    
    def shutdown(self):
        rospy.loginfo("노드 종료 중...")

        #타이머 정리
        if hasattr(self, 'memory_check_timer'):
            self.memory_check_timer.shutdown()

        if hasattr(self, 'timeout_check_timer'):
            self.timeout_check_timer.shutdown()

        # FFmpeg 프로세스 정리
        with self.ffmpeg_lock:
            if self.ffmpeg is not None:
                try:
                    self.ffmpeg.terminate()
                    self.ffmpeg.wait(timeout=3)
                except:
                    try:
                        self.ffmpeg.kill()
                    except:
                        pass
                self.ffmpeg = None

        if self.debug_mode:
            try:
                cv2.destroyAllWindows()
            except:
                pass

def main():
    streamer = YOLOIRRtmpStreamer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료됨")

if __name__ == '__main__':
    main()


#!/usr/bin/env bash
# start_all.sh

# Ctrl+C 받으면 background job 전부 종료
trap 'echo "Stopping all..."; kill $(jobs -p) 2>/dev/null; exit' SIGINT

# 1) YOLO
(
  cd /home/ubuntu/yolo/ultralytics/y11_c202 \
  && python3 detect_y11.py
) &

# 2) ros2mqtt
(
  python3 /home/ubuntu/ec2/ros2mqtt/ros2mqtt.py
) &

# 3) joystick_v2
(
  cd /home/ubuntu/catkin_ws \
  && source devel/setup.bash \
  && rosrun gcs_esp32_controller joystick_v2.py
) &

# 4) robot_tilting_hello launch
(
  cd /home/ubuntu/catkin_ws \
  && source devel/setup.bash \
  && roslaunch gcs_esp32_controller robot_tilting_hello.launch
) &

# 5초 대기 후 GCS UI 실행
sleep 5
(
  cd /home/ubuntu/S12P31C202/qt_ws \
  && source devel/setup.bash \
  && rosrun gcs_ui gcs_ui
) &

wait


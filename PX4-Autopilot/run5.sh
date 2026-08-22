#!/bin/bash
# 터미널 1 - Gazebo
gnome-terminal -- bash -c "cd ~/Desktop/com1/PX4-Autopilot && make px4_sitl gz_x500; exec bash"
sleep 15
# 터미널 2 - MicroXRCE DDS Agent
gnome-terminal -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 2
# 터미널 3 - 카메라 브릿지
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/gz_ws/install/setup.bash && ros2 run ros_gz_image image_bridge /camera/imx219; exec bash"
sleep 2
# 터미널 4 - 라인 스캐닝
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/Desktop/com1/ws_px4_msgs/install/setup.bash && cd ~/Desktop/com1/PX4-Autopilot/navigation && python3 camera_line_scanning.py; exec bash"


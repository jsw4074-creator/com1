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
sleep 5
# 터미널 5 - spawn_entity 브릿지
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/gz_ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/SpawnEntity; exec bash"
sleep 3
# 터미널 6 - ArUco 마커 스폰
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/gz_ws/install/setup.bash && cd ~/Desktop/com1/PX4-Autopilot/navigation && python3 spawn_aruco_marker.py; exec bash"
sleep 3
# 터미널 7 - follow_waypoints (로그/데이터 창)
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/Desktop/com1/ws_px4_msgs/install/setup.bash && cd ~/Desktop/com1/PX4-Autopilot/navigation && waypoint_follower_mode1_only; exec bash"
sleep 3
# 터미널 8 - mode_commander (명령어 입력 창)
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/Desktop/com1/ws_px4_msgs/install/setup.bash && cd ~/Desktop/com1/PX4-Autopilot/navigation && python3 mode_commander.py; exec bash"

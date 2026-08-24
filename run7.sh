#!/bin/bash

unset GZ_SIM_RESOURCE_PATH
unset IGN_GAZEBO_RESOURCE_PATH


# 터미널 1 - Gazebo / PX4 SITL
gnome-terminal -- bash -c "
cd ~/Desktop/com1/PX4-Autopilot &&
make px4_sitl gz_x500;
exec bash
"

sleep 15

# 터미널 2 - MicroXRCE DDS Agent
gnome-terminal --title="MicroXRCE DDS Agent" -- bash -c "
~/Desktop/com1/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888;
exec bash
"


sleep 2

# 터미널 3 - 카메라 브릿지
gnome-terminal --title="Camera Bridge" -- bash -c "
source /opt/ros/humble/setup.bash &&
source ~/gz_ws/install/setup.bash &&
ros2 run ros_gz_image image_bridge /camera/imx219;
exec bash
"

sleep 2

# 터미널 4 - SpawnEntity 브릿지
gnome-terminal --title="SpawnEntity Bridge" -- bash -c "
source /opt/ros/humble/setup.bash &&
source ~/gz_ws/install/setup.bash &&
ros2 run ros_gz_bridge parameter_bridge \
/world/default/create@ros_gz_interfaces/srv/SpawnEntity;
exec bash
"

sleep 3

# 터미널 5 - ArUco 마커 스폰
gnome-terminal --title="ArUco Marker Spawn" -- bash -c "
source /opt/ros/humble/setup.bash &&
source ~/gz_ws/install/setup.bash &&
cd ~/Desktop/com1/PX4-Autopilot/navigation &&
python3 spawn_aruco_marker.py;
exec bash
"

sleep 3

# 터미널 6 - 메인 비행 코드 + 디버그 창
gnome-terminal --title="Main Flight + Debug" -- bash -c "
source /opt/ros/humble/setup.bash &&
source ~/Desktop/com1/ws_px4_msgs/install/setup.bash &&
cd ~/Desktop/com1/PX4-Autopilot/New_code &&
python3 main.py;
exec bash
"

sleep 2

# 터미널 7 - 모드 커맨더
gnome-terminal --title="Mode Commander" -- bash -c "
source /opt/ros/humble/setup.bash &&
source ~/Desktop/com1/ws_px4_msgs/install/setup.bash &&
cd ~/Desktop/com1/PX4-Autopilot/New_code &&
python3 mode_commander.py;
exec bash
"

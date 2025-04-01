# MREN203_MechatronicsDesignII
MREN 203 - Mechatronics and Robotics Design II

Authors: Sebastien Sauter, Jamie Strain, and Marlow Gaddes

## Relevant Commands

ssh into the pi from cmd `ssh -l group10 192.168.50.210` 

tailscale ip address `100.97.116.103`

source ROS2 environment `source /opt/ros/humble/setup.bash`

source install to load env. variables `source install/setup.bash`

source bash to the terminal `source .bashrc`

list serial id `ls /dev/serial/by-id/` 
>arduino serial id `usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01`

open serial monitor on pi `screen /dev/serial/by-id/<usb_port_id> <baudrate>` 
>`screen /dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01 9600` 
>use ctrl + a, then k, then y to kill

launch LiDAR `ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0 -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:=Standard -p serial_baudrate:=115200`

Launch URDF `ros2 run robot_state_publisher robot_state_publisher bot_10.urdf` and `ros2 run joint_state_publisher_gui joint_state_publisher_gui`

Rviz `rviz2` open bot_10.rviz

launch gazebo `ing gazebo blank.sdf` (replace blank with map sdf file)

full read and write access for the serial port `sudo chmod 777 /dev/ttyACM0`

basic ROS2 commands
```
ros2 node list
ros2 topic list
ros2 topic echo <topic_name>
python3 <node_name>
ros2 launch <package_name> <node_name.launch.py>
EX: ros2 launch serial_driver serial_driver_bridge_node.launch.py
```

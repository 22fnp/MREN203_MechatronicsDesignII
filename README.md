# MREN203_MechatronicsDesignII
MREN 203 - Mechatronics and Robotics Design II

Authors: Sebastien Sauter, Jamie Strain, and Marlow Gaddes

## Relevant Commands

ssh into the pi from cmd `ssh -l group10 192.168.50.210` 

tailscale ip address `100.97.116.103`

source ROS2 environment `source /opt/ros/humble/setup.bash`

list serial id `ls /dev/serial/by-id/` 
>arduino serial id `usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01`

open serial monitor on pi `screen /dev/serial/by-id/<usb_port_id> <baudrate>` 
>`screen /dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01 9600` 
>use ctrl + a, then k, then y to kill

launch LiDAR and view in Rviz `ros2 launch sllidar_ros2 view_sllidar_a1_launch.py`

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

`source install/setup.bash`

# MREN203_MechatronicsDesignII
MREN 203 - Mechatronics and Robotics Design II

Authors: Sebastien Sauter, Jamie Strain, and Marlow Gaddes

## Relevant Commands

To ssh into the pi from cmd `ssh -l group10 192.168.50.210` 

`source /opt/ros/humble/setup.bash`

`ls /dev/serial/by-id/`

`ros2 run serial_driver serial_bridge --ros-args -p serial_port:=/dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01 -p baud_rate:=9600 -p flow_control:=none -p parity:=none -p stop_bits:=1`


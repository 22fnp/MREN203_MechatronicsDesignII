# MREN203_MechatronicsDesignII
MREN 203 - Mechatronics and Robotics Design II

Authors: Sebastien Sauter, Jamie Strain, and Marlow Gaddes

## Relevant Commands

ssh into the pi from cmd `ssh -l group10 192.168.50.210` 

source ROS2 environment `source /opt/ros/humble/setup.bash`

list serial id `ls /dev/serial/by-id/` = `usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01`

open serial monitor on pi `screen /dev/serial/by-id/<usb_port_id> <baudrate>` = `screen /dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01 9600`

to run serial bridge node `ros2 run serial_driver serial_bridge --ros-args --params-file ./src/transport_drivers/serial_driver/params/example.params.yaml`

`ros2 run serial_driver serial_bridge --ros-args -p serial_port:=/dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01 -p baud_rate:=9600 -p flow_control:=none -p parity:=none -p stop_bits:=1`


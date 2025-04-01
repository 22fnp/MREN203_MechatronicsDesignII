import rclpy
from rclpy.node import Node
import array

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

import json
import serial
import time

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_motor_driver_node')

        self.ser = serial.Serial("/dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_24569C6163A133BEDC04-if01",115200)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'keyboard_controller',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        try:
            # Convert the Float32MultiArray to a simple list
            data = list(msg.data)

            # Create json msg
            json_write = json.dumps({
                't':data[0] if len(data)>0 else 0.0, # Translational Speed
                'a':data[1] if len(data)>1 else 0.0  # Angular Rate
            })

            self.ser.write((json_write + '\n').encode())
            time.sleep(0.2)
            
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")

        finally:
            self.get_logger().info(f"Sent JSON: {json_write}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriber()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()
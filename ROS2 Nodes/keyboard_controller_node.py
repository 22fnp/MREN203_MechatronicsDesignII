import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import sys
import tty
import termios
import select

class KeyboardControllerPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_controller_node')

        self.setpoint_translational_speed = 0.7 # [m/s]
        self.setpoint_angular_rate = 4 # [rad/s]
        self.current_translational_speed = 0.0
        self.current_angular_rate = 0.0

        # Dictionary, stores velocity data for each keyboard input
        self.key_library = {
            'w' : lambda : (self.setpoint_translational_speed, 0.0),
            's' : lambda : (-self.setpoint_translational_speed, 0.0),
            'a' : lambda : (0.0, self.setpoint_angular_rate),
            'd' : lambda : (0.0, -self.setpoint_angular_rate),
            ' ' : lambda : (0.0, 0.0),
            'x' : 'exit'
        }

        self.publisher = self.create_publisher(
            Float32MultiArray,
            'keyboard_controller',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback())

    def get_key(self):
        tty.setcbreak(sys.stdin.fileno())  # define terminal as a stream of input
        dr = select.select([sys.stdin], [], [], 0.1)[0]  # Check if input is available
        if dr:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                #tty.setraw(fd)
                key = sys.stdin.read(1)  # Read one character
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return key
        return None  # No key was pressed

    def timer_callback(self):
        try:
            time_last = time.monotonic()
            period = 0.2
            while True:
                key = self.get_key()
                if key in self.key_library:
                    movement = self.key_library[key] # Set the current movements based on the key
                    if movement == 'exit':
                        break # Break when 'x' is pressed, loop inf. if 'x' is not pressed
                    else:
                        self.current_translational_speed, self.current_angular_rate = movement()

                msg = Float32MultiArray()
                msg.data = [self.current_translational_speed, self.current_angular_rate]

                # self.publisher.publish(msg)
                time_now = time.monotonic()
                if ((time_now - time_last) > period):
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Translational = {msg.data[0]}")
                    self.get_logger().info(f"Rotational = {msg.data[1]}")
                    time_last = time.monotonic()

        except Exception as e:
            print(f"\nError: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControllerPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

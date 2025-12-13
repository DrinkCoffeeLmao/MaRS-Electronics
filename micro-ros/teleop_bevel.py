#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import sys, select, termios, tty

class TeleopRover(Node):
    def __init__(self):
        super().__init__('teleop_rover')
        self.publisher_ = self.create_publisher(Int16, 'bevel_cmd', 10)
        self.get_logger().info("Teleop Rover started.")
        self.get_logger().info("Use keys: f=forward, b=backward, l=left, r=right, s=stop, q=quit")

    def get_key(self):
        # Non-blocking keyboard read
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        key_mapping = {
            'f': 1,   # scoop up
            'b': 2,   # scoop down
            'k': 3,   # stop
            'l': 4,   #rotate right
            'r': 5,   # rotate left
            'u': 6,   #stop steer
        }

        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'q':
                    self.get_logger().info("Quit teleop.")
                    break

                if key in key_mapping:
                    msg = Int16()
                    msg.data = key_mapping[key]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published cmd: {msg.data} ({key})")

        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopRover()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

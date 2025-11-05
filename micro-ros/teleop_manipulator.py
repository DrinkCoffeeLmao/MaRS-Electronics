#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys, select, termios, tty

class TeleopManipulator(Node):
    def __init__(self):
        super().__init__('teleop_manipulator')
        self.publisher_ = self.create_publisher(Int32, 'manipulator_cmd', 10)
        self.get_logger().info("Teleop manipulator started.")
        self.get_logger().info("Use keys a=base++ d=base-- w=l1++ s=l1-- z=l2++ x=l2-- q=quit")

    def get_key(self):
        # Non-blocking keyboard read
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        key_mapping = {
       	    'p': 0,   #stop	
            'a': 1,   # base++
            'd': 2,   # base--
            'w': 3,   # L1++
            's': 4,   # L1--
            'z': 5,   # L2++
            'x': 6,   #L2--
        }

        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'q':
                    self.get_logger().info("Quit teleop.")
                    break

                if key in key_mapping:
                    msg = Int32()
                    msg.data = key_mapping[key]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published cmd: {msg.data} ({key})")

        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopManipulator()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

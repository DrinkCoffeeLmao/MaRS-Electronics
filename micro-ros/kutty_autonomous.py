#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import time

CMD_DOWN = 9
CMD_UP   = 10
CMD_OPEN = 7
CMD_CLOSE = 8

class AutoLimitController(Node):

    def __init__(self):
        super().__init__('auto_limit_controller')

        self.cmd_pub = self.create_publisher(Int16, 'rover_cmd', 10)
        self.limit_sub = self.create_subscription(
            Int16,
            'limit_switch',
            self.limit_callback,
            10
        )

        self.limit_hit = False
        self.action_done = False

        self.state = 0
        self.start_time = time.time()

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Auto limit controller started")

    def limit_callback(self, msg):
        # DO NOT CHANGE
        self.limit_hit = (msg.data == 0)

    def publish_cmd(self, cmd):
        msg = Int16()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        time.sleep(0.1)   # small delay between publishes

    def control_loop(self):
        now = time.time()

        # 1️⃣ Go DOWN for 1 second
        if self.state == 0:
            self.publish_cmd(CMD_DOWN)
            if now - self.start_time >= 1.5:
                time.sleep(1.0)   # ⏸ delay after sequence
                self.state = 1
                self.start_time = time.time()

        # 2️⃣ Open scoop for 2 seconds
        elif self.state == 1:
            self.publish_cmd(CMD_OPEN)
            if now - self.start_time >= 2:
                time.sleep(1.0)   # ⏸ delay after sequence
                self.state = 2

        # 3️⃣ Go DOWN till limit switch is pressed
        elif self.state == 2:
            if not self.limit_hit:
                self.publish_cmd(CMD_DOWN)
            else:
                time.sleep(1.0)   # ⏸ delay after limit hit
                self.state = 3
                self.start_time = time.time()

        # 4️⃣ Wait 2 seconds after limit hit
        elif self.state == 3:
            if now - self.start_time >= 1.0:
                time.sleep(1.0)   # ⏸ delay after wait
                self.state = 4
                self.start_time = time.time()

        # 5️⃣ Close scoop for 2 seconds
        elif self.state == 4:
            self.publish_cmd(CMD_CLOSE)
            if now - self.start_time >= 2.5:
                time.sleep(1.0)   # ⏸ delay after sequence
                self.state = 5
                self.start_time = time.time()

        # 6️⃣ Move UP for 3 seconds
        elif self.state == 5:
            self.publish_cmd(CMD_UP)
            if now - self.start_time >= 3.0:
                time.sleep(1.0)   # ⏸ final delay
                self.timer.cancel()
                self.action_done = True


def main():
    rclpy.init()
    node = AutoLimitController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3

import sys

from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, '/vacuum_gripper/switch')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, status):
        self.req.data = status
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    status = True
    response = minimal_client.send_request(status)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
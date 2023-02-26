import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Empty, '/clear')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Empty.Request()

    def clear_frame(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.clear_frame()
    print(response)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
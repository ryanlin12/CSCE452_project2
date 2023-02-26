import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetPen.Request()

    # def send_request(self, r, g, b, width, off): #example with user input
    #     self.req.r = r
    #     self.req.g = g
    #     self.req.b = b
    #     self.req.width = width
    #     self.req.off = off
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()

    def set_line_color(self): #actual code (color is set to white)
        self.req.r = 255
        self.req.g = 255
        self.req.b = 255
        self.req.width = 1
        self.req.off = 0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    #response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))
    response = minimal_client.set_line_color()
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    print(response)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
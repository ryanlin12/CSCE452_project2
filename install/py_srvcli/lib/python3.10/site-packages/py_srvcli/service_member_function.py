from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetPen, '/turtle1/set_pen', self.call_setpen_service)

    # def add_two_ints_callback(self, request, response):
    #     response.sum = request.a + request.b
    #     self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    #     return response
    # def call_setpen_service(self, r, g, b, width, off):
    #     response.r = 
    def call_setpen_service(self, request, response):
        response.r = request.r
        response.g = request.g
        response.b = request.b
        response.width = request.off
        response.off = request.off
        self.get_logger().info('Incoming request\nr: %d g: %d b: %d width: %d off: %d' % (request.r, request.g, request.b, request.width, request.off))

def main():
    rclpy.init()
    
    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
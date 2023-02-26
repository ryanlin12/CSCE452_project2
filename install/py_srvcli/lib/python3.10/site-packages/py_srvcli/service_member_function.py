from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetPen, '/turtle1/set_pen', self.confirm_setpen_service)

    # def add_two_ints_callback(self, request, response):
    #     response.sum = request.a + request.b
    #     self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    #     return response
    # def call_setpen_service(self, r, g, b, width, off):
    #     response.r = 
    def confirm_setpen_service(self, request, response): #setpen does not have a return value from the server so all were doing here is confirming that it actually happened from the client
        print (request)
        return response #returns an empty response
        

def main():
    rclpy.init()
    
    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
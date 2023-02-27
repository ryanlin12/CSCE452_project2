import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

#DONT NEED
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, '/clear', self.confirm_clear)

    def confirm_clear(self, request, response):
        print(request)
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
import time
from turtlesim.srv import SetPen
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType

#!/usr/bin/env python3

from math import pi

SQRT2 = 1.41421
TOLERANCE = 0.03
ANGLE_TOLERANCE = 2*pi/180
SIH = 5.0

class Move:
    def __init__(self, action, x, y, theta):
        self.action = action
        self.x = x
        self.y = y
        self.theta = theta


class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.x = 5.5
        self.y = 5.5
        self.theta = 0

        #setting the parameters
        self.cli2 = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available to get parameters, waiting again...')
        self.req2 = SetParameters.Request()

        #set pen
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available to set pen, waiting again...')
        self.req1 = SetPen.Request()

        

        #create publisher to cmd_vel topic to control the turtle
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 1000)
        #self.pose_publisher = self.create_publisher(Pose, '/turtle1/pose', 1000)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1000)
        self.timer = self.create_timer(10, self.timer_callback)
        self.move_list = []

        #raw_t = [(74, 71), (423, 71), (423, 167), (356, 167), (356, 131), (282, 131), (282, 360), (319, 360), (319, 427), (171, 427), (171, 360), (215, 360), (215, 130), (141, 130), (141, 167), (75, 167)]
        
        #cooked_t = [(11 * x1/500, 11 * (500-y1)/500) for (x1, y1) in raw_t]

        #print(cooked_t)

        self.square = [(5.5, 5.5), (3, 3), (3, 8), (8, 8), (8, 3), (3, 3)]

        self.last_twistmsg = -1 #-1 is before initialization, 1 is rotating, 2 is linear


        self.current_point = 0
        


    def timer_callback(self):
        print('timer callback called')
        twist_msg = Twist()
        twist_msg.linear.x = float(1.0)
        twist_msg.angular.z = float(pi/4)
        self.cmd_vel_publisher.publish(twist_msg)

    def pose_callback(self, msg):
        print('pose callback called')
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        

    def set_pen(self,off):
        print("changing pen color")
        self.req1.r = 255
        self.req1.g = 255
        self.req1.b = 255
        self.req1.off = off
        self.future = self.cli.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_bckgrnd_color(self, a, b):
        # self.req2.parameters = [a, b]
        # self.future = self.cli2.call_async(self.req2)
        # rclpy.spin_until_future_complete(self, self.future)ffd
        # return self.future.result()
        new_param_value = ParameterValue(integer_value= b, type=ParameterType.PARAMETER_INTEGER)
        self.req2.parameters = [Parameter(name= a, value=new_param_value)]
        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController() #creation of the node
    #turtle_controller.set_bckgrnd_color('background_r', 13)
    turtle_controller.set_pen(1)
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    #rclpy.spin(turtle_controller)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
    




#use spin to call a callback over and over again that publishes cmd_vel messages that cointrols the turtle into logo
#feedback from pose is helpful to get movement to be accurate
#i know where i am right now, what command do i give to direct the turtle in the direction (switch between pose and cmd_vel)
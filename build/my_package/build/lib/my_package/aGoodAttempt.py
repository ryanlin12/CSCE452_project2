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
ANGLE_TOLERANCE = pi/180
SIH = 5.0



class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.x = 5.5
        self.y = 5.5
        self.theta = 0

        #setting the pen
        
        
        self.cli2 = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available to get parameters, waiting again...')
        self.req2 = SetParameters.Request()

        
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available to set pen, waiting again...')
        self.req1 = SetPen.Request()

        

        #create publisher to cmd_vel topic to control the turtle
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 1000)
        self.pose_publisher = self.create_publisher(Pose, '/turtle1/pose', 1000)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1000)

        #self.move_list = []

        #raw_t = [(74, 71), (423, 71), (423, 167), (356, 167), (356, 131), (282, 131), (282, 360), (319, 360), (319, 427), (171, 427), (171, 360), (215, 360), (215, 130), (141, 130), (141, 167), (75, 167)]
        
        #cooked_t = [(11 * x1/500, 11 * (500-y1)/500) for (x1, y1) in raw_t]

        #print(cooked_t)

        #self.square = [(3, 3), (3, 8), (8, 8), (8, 3), (3, 3)]

        #self.last_twistmsg = None


        #self.square = [(5.5, 5.5)]
        #self.square.extend(cooked_t)
        #self.square.append(cooked_t[0])
        self.current_point = 0
        

        #TODO: might have to edit package.xml to include geometrymsgs (would be in ros2 turtorial 7 by robotics back-end)
        # self.timer = self.create_timer(0.5, self.move_turtle)
        #create client requests to set_pen service to change the color of the pen
        # self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('SetPen service not available, waiting again...')
        # self.req = SetPen.Request()

    '''
    def vectortwister(self, v1, v2):
        # Define two vectors
        # v1 = [1, 2, 3]
        # v2 = [-2, 1, 4]

        # Calculate the dot product between the vectors
        dot_product = sum(v1[i] * v2[i] for i in range(len(v1)))

        # Calculate the magnitude of each vector
        magnitude_v1 = math.sqrt(sum(i ** 2 for i in v1))
        magnitude_v2 = math.sqrt(sum(i ** 2 for i in v2))

        # Calculate the angle in radians using the dot product and magnitudes
        angle = math.acos(dot_product / (magnitude_v1 * magnitude_v2))

        # Return the angle in radians
        return angle
    '''

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        #print(self.theta)
        twist_msg = Twist()
       

        # target = self.square[self.current_point]

        

        # if abs(self.x - target[0]) <= TOLERANCE and abs(self.y - target[1]) <= TOLERANCE:
        #     self.current_point += 1
        #     #print('upping point')
        #     return
        # else:
        #     vector_curr_to_target = (target[0] - self.x, target[1] - self.y)
        #     #dist = math.sqrt(vector_curr_to_target[0] ** 2 + vector_curr_to_target[1] ** 2)
        #     # target_angle = math.acos(vector_curr_to_target[0] / dist)
        #     target_angle = math.atan2(vector_curr_to_target[1], vector_curr_to_target[0])
        #     target_angle = target_angle % (2 * pi)
        #     if target_angle > pi:
        #         target_angle -= 2 * pi

        #     if self.rotate(self.theta, target_angle):
        #         #print('rotated')
        #         twist_msg.angular.z = float(0.0)
        #         twist_msg.linear.x = float(2.0) #float(vector_curr_to_target[0] * 10)
        #         # twist_msg.linear.y = float(0.0) #float(vector_curr_to_target[1] * 10)
        #     else:
        #         twist_msg.angular.z = float(2)

        #     if self.last_twistmsg is None:
        #         self.cmd_vel_publisher.publish(twist_msg)
        #     else:
        #         if self.last_twistmsg.linear.x == twist_msg.linear.x and self.last_twistmsg.angular.z == twist_msg.angular.z:
        #             return
        #         else:
        #             self.cmd_vel_publisher.publish(twist_msg)
        #     self.last_twistmsg = twist_msg

                
                
                
    
      

    def rotate(self, current_theta, target_theta):
        #self.get_logger().info(f"Turtle's current position: x = {self.x:.5f}, y = {self.y:.5f}, theta = {self.theta:.5f}")
        #elf.get_logger().info(f"Turtle's current angles: current = {current_theta:.5f}, target = {target_theta:.5f}")
        return abs(current_theta - target_theta) < ANGLE_TOLERANCE

    def move_test(self):
        for i in range(0, 20):
            twist_msg = Twist()
            twist_msg.linear.x = float(i - (i - 1))
            twist_msg.linear.y = float(i - (i - 1))
            twist_msg.angular.z = float(i - (i - 1))
            self.cmd_vel_publisher.publish(twist_msg)
            
            
        
            print('I WANNA DIE %d' % i)
        

    def move_to(self, x1, y1, x2, y2, speed): #implement pose subs
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)
        speed = float(speed)
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        angle = math.atan2(y2 - y1, x2 - x1)
        
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0

 
        while distance > 0.01:
            self.cmd_vel_publisher.publish(twist_msg)
            distance = math.sqrt((x2 - self.x) ** 2 + (y2 - self.y) ** 2)
            angle_to_goal = math.atan2(y2 - self.y, x2 - self.x)
            angular_speed = 4.0 * (angle_to_goal - self.theta)
            twist_msg.linear.x = speed
            twist_msg.angular.z = angular_speed
            self.cmd_vel_publisher.publish(twist_msg)
            print('I WANNA DIE %.f' % distance)
         

    
    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = 0.2  # set the linear velocity to 0.2 m/s
        distance_moved = 0
        loop_rate = self.create_rate(10)  # create a loop rate of 10 Hz
        while distance_moved < distance:
            self.cmd_vel_publisher.publish(twist)
            loop_rate.sleep()
            distance_moved += 0.2 / 10  # update the distance moved
        twist.linear.x = 0  # stop the robot
        self.cmd_vel_publisher.publish(twist)

    def turn(self, angle):
        twist = Twist()
        twist.angular.z = 0.4  # set the angular velocity to 0.4 rad/s
        angle_turned = 0
        loop_rate = self.create_rate(10)  # create a loop rate of 10 Hz
        while angle_turned < angle:
            self.cmd_vel_publisher.publish(twist)
            loop_rate.sleep()
            angle_turned += 0.4 / 10  # update the angle turned
        twist.angular.z = 0  # stop the robot
        self.cmd_vel_publisher.publish(twist)


    # def move(self, linear_speed, angular_speed, duration):
    #     msg = Twist()
    #     msg.linear.x = float(linear_speed)
    #     msg.angular.z = float(angular_speed)
    #     end_time = self.get_clock().now().nanoseconds*1e9 + duration
    #     while self.get_clock().now().nanoseconds*1e9 < end_time:
    #         self.cmd_vel_publisher.publish(msg)
    #         self.sleep()

    # def set_param(self, name, val):rValue(integer_value= b, type=ParameterType.PARAMETER_INTEGER)
        self.req2.parameters = [Parameter(name= a, value=new_param_value)]
    #     self.req.parameters = [Parameter ]

    # def set_line_color(self, off): #actual code (color is set to white)
    #     self.req.r = 255
    #     self.req.g = 255
    #     self.req.b = 255
    #     self.req.width = 1
    #     self.req.off = off
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()


    def run(self): #this is the shape of the a&m logo
        #Letter T

        #while time < 3 seconds #the first line
            #self.timer = create_timer(0.1, self.move_turtle)
        #while time < 2 seconds #the first turn
            #self.timer - create_timer(0.1, self.turn_turtle)
        #self.move_test()
        #self.draw_line(1)
        self.move_to(0, 0, 5, 5, 3)
        #self.move_turtle_comp(0, 2)
        
        
        # for i in range(4):
        #     self.move_turtle()
        #     self.turn_turtle()
        self.timer = self.create_timer(0.01, self.move_turtle)
        

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
    turtle_controller.set_pen(0)
    #turtle_controller.run()
    # move forward for 1 meter
    turtle_controller.move_forward(5000)

    # turn 90 degrees to the right
    turtle_controller.turn(1.57)  # 1.57 radians = 90 degrees

    # move forward for 1 meter
    turtle_controller.move_forward(20)

    # turn 90 degrees to the right
    turtle_controller.turn(1.57)  # 1.57 radians = 90 degrees

    # move forward for 1 meter
    turtle_controller.move_forward(50)

    # turn 90 degrees to the right
    turtle_controller.turn(1.57)  # 1.57 radians = 90 degrees

    # move forward for 1 meter
    turtle_controller.move_forward(20)
    turtle_controller.destroy_node()
    #rclpy.spin(turtle_controller)
    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
    




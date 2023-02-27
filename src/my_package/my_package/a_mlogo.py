import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
import time

#!/usr/bin/env python3

from math import pi

SQRT2 = 1.41421
TOLERANCE = 0.01
ANGLE_TOLERANCE = 0.01
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
        #create publisher to cmd_vel topic to control the turtle
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(Pose, '/turtle1/pose', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1)

        self.move_list = []

        #raw_t = [(74, 71), (423, 71), (423, 167), (356, 167), (356, 131), (282, 131), (282, 360), (319, 360), (319, 427), (171, 427), (171, 360), (215, 360), (215, 130), (141, 130), (141, 167), (75, 167)]
        
        #cooked_t = [(11 * x1/500, 11 * (500-y1)/500) for (x1, y1) in raw_t]

        #print(cooked_t)

        self.square = [(5.5, 5.5), (3, 3), (3, 8), (8, 8), (8, 3), (3, 3)]
        #self.square = [(5.5, 5.5)]
        #self.square.extend(cooked_t)
        #self.square.append(cooked_t[0])
        #self.current_point = 1
        

        #TODO: might have to edit package.xml to include geometrymsgs (would be in ros2 turtorial 7 by robotics back-end)
        # self.timer = self.create_timer(0.5, self.move_turtle)
        #create client requests to set_pen service to change the color of the pen
        # self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('SetPen service not available, waiting again...')
        # self.req = SetPen.Request()


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

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        target = self.square[self.current_point]
        twist_msg = Twist()

        curr_vector = (target[0] - self.square[self.current_point - 1][0], target[1] - self.square[self.current_point - 1][1])
        curr_theta = self.vectortwister(list(curr_vector), [1, 0])
        print(curr_theta, 'cc')
        
        #print("ABSX: ", abs(self.x - target[0]), " ABSY: ", abs(self.y - target[1]))
        if abs(self.x - target[0]) <= TOLERANCE and abs(self.y - target[1]) <= TOLERANCE:
            
            twist_msg.linear.x = float(0.0)
            twist_msg.linear.y = float(0.0)
            self.current_point += 1
        else:
            twist_msg.linear.x = float(curr_vector[0]/SIH)
            twist_msg.linear.y = float(curr_vector[1]/SIH)

            self.get_logger().info("i %.f %.f" % (twist_msg.linear.x, twist_msg.linear.y))
            #self.get_logger().info(f"Turtle's current position: x = {self.x:.2f}, y = {self.y:.2f}, theta = {self.theta:.2f}")
        
        self.cmd_vel_publisher.publish(twist_msg)


    def rotate(self, target_theta):
        #use the dot product divided by two mags together
        pass

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
         

    

    # def move(self, linear_speed, angular_speed, duration):
    #     msg = Twist()
    #     msg.linear.x = float(linear_speed)
    #     msg.angular.z = float(angular_speed)
    #     end_time = self.get_clock().now().nanoseconds*1e9 + duration
    #     while self.get_clock().now().nanoseconds*1e9 < end_time:
    #         self.cmd_vel_publisher.publish(msg)
    #         self.sleep()

    # def set_param(self, name, val):
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
        

        

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController() #creation of the node
    
    #turtle_controller.run()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    #rclpy.spin(turtle_controller)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
    




# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from math import sqrt, pi, atan2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType


class TurtleController(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10) #publish to turtle1/cmd topic
        timer_period = 0.1  # second
        self.timer = self.create_timer(timer_period, self.move) #callback is called every second
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1000)
        self.flag = False
        self.i = 0
        self.x = 0
        self.y = 0
        self.theta = 0


        #set pen
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available to set pen, waiting again...')
        self.req1 = SetPen.Request()

        #setting the parameters
        self.cli2 = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available to get parameters, waiting again...')
        self.req2 = SetParameters.Request()

        #clear backgorund
        self.cli3 = self.create_client(Empty, '/clear')
        while not self.cli3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req3 = Empty.Request()

        #square points
        #self.square = [(3, 3), (3, 8), (8, 8), (8, 3), (3, 3), (3, 4)]
        raw_a = [(-67,171),(126,171),(126,197),(119,197),(160,277),(173,277),(173,311),(114,311),(114,278),(121,278),(113,264),(80,264),(73,278),(80,278),(80,311),(20,311),(20,278),(40,278),(73,197),(66,197),(67,171), (-98,218),(85,238),(109,238), (97,215)]
        raw_t = [(-74, 71), (74, 71), (423, 71), (423, 167), (356, 167), (356, 131), (282, 131), (282, 360), (319, 360), (319, 427), (171, 427), (171, 360), (215, 360), (215, 130), (141, 130), (141, 167), (74, 167), (74, 71)]
        raw_m = [(-274,171),(330,171),(347,229),(381,171),(427,171),(427,197),(414,197),(414,278),(427,278),(427,311),(374,311),(374,278),(387,278),(387,219),(354,290),(314,219),(314,278),(327,278),(327,311),(274,311),(274,278),(287,278),(287,197),(274,197), (274,171), (-1,0), (0,0)]
        
        raw_t = [(x1 - 25, y1 - 30) for (x1, y1) in raw_t]
        
        cooked_t = [(11 * x1/500, 11 * (500-y1)/500) for (x1, y1) in raw_t]
        cooked_m = [(11 * x1/500, 11 * (500-y1)/500) for (x1, y1) in raw_m]
        cooked_a = [(11 * x1/500, 11 * (500-y1)/500) for (x1, y1) in raw_a]
        
        self.cooked_tamu = cooked_a
        self.cooked_tamu.extend(cooked_t)
        self.cooked_tamu.extend(cooked_m)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def dist(self, target):
        return sqrt(pow(target.x - self.x, 2) + pow(target.y - self.y, 2))

    def linear_vel(self, target, constant=2):
        return constant * self.dist(target)

    def steer(self, target):
        return atan2(target.y - self.y, target.x - self.x)

    def angular_vel(self, target, constant=3):
        return constant*(self.steer(target)-self.theta)
    
    def set_pen(self,off):
        self.req1.r = 255
        self.req1.g = 255
        self.req1.b = 255
        self.req1.off = off
        self.req1.width = 3
        self.future = self.cli.call_async(self.req1)
        #rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_bckgrnd_color(self, a, b):
        # self.req2.parameters = [a, b]
        # self.future = self.cli2.call_async(self.req2)
        # rclpy.spin_until_future_complete(self, self.future)ffd
        # return self.future.result()
        new_param_value = ParameterValue(integer_value= int(b), type=ParameterType.PARAMETER_INTEGER)
        self.req2.parameters = [Parameter(name= a, value=new_param_value)]
        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def clear_frame(self):
        self.future = self.cli3.call_async(self.req3)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    #SET_BCKGRND_COLOR ANOTHER METHOD ATTEMPT:
    '''
    def set_bckgrnd_color(self, a, b):
        new_param_value = ParameterValue(type=ParameterValue.PARAMETER_INTEGER, integer_value=int(b))
        self.req2.parameters = [Parameter(name=a, value=new_param_value)]
        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    '''

    def move(self):
        
        #lift pen: self.set_pen(1)
        self.set_pen(1 if self.cooked_tamu[self.i][0] < 0 else 0)
        #self.set_pen(1)
        target = Pose()
        target.x = float(abs(self.cooked_tamu[self.i][0]))
        target.y = float(self.cooked_tamu[self.i][1])
        target.theta = self.steer(target)   

        dist_tol = 0.1
        ang_tol = 0.01
        twist_msg = Twist()
        if (self.i == len(self.cooked_tamu) - 1):
            quit()
        if abs(self.steer(target)-self.theta) > ang_tol:
            twist_msg.linear.x = float(0.0)
            twist_msg.angular.z = self.angular_vel(target)
        else:
            twist_msg.angular.z = 0.0
            if self.dist(target) >= dist_tol:
                twist_msg.linear.x = self.linear_vel(target)
            else:
                twist_msg.linear.x = float(0.0)
                self.flag = True
        
        if self.flag:
            twist_msg.angular.z = target.theta - self.theta
            if abs(target.theta - self.theta) <= ang_tol:
                #quit()
                self.i += 1
                self.flag = False
                return
            

        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TurtleController()
    minimal_publisher.set_bckgrnd_color('background_r', 13)
    minimal_publisher.set_bckgrnd_color('background_g', 13)
    minimal_publisher.set_bckgrnd_color('background_b', 13)
    minimal_publisher.clear_frame()
    minimal_publisher.set_pen(1)
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pi

class SquareTurtle(Node):
    def __init__(self, length):
        super().__init__('square_turtle')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, 'turtle1/pose', self.callback_pose, 10)
        self.current_pose = Pose()
        self.target_pose = Pose()
        self.target_pose.theta = 0.0
        self.length = length
        self.side = 0

    def callback_pose(self, msg):
        self.current_pose = msg

    def move_turtle(self):
        twist_msg = Twist()

        while True:
            # Check if the turtle is close to a wall
            if self.current_pose.x < 0.5 or self.current_pose.x > 10.5 or self.current_pose.y < 0.5 or self.current_pose.y > 10.5:
                # Turn the turtle around
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = pi
                self.publisher.publish(twist_msg)
                rclpy.spin_once(self)
            else:
                # Move the turtle forward
                twist_msg.linear.x = 1.0
                twist_msg.angular.z = 0.0
                self.publisher.publish(twist_msg)
                rclpy.spin_once(self)

            # Check if the turtle has moved the required distance
            if self.get_distance(self.current_pose, self.target_pose) >= self.length:
                # Turn the turtle to the next side
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = pi/2
                self.publisher.publish(twist_msg)
                rclpy.spin_once(self)

                # Update the target pose and side number
                self.target_pose = self.current_pose
                self.target_pose.theta += pi/2
                self.side += 1

            # Check if the turtle has completed all sides of the square
            if self.side == 4:
                # Stop the turtle and exit the loop
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher.publish(twist_msg)
                break

    def get_distance(self, pose1, pose2):
        return ((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)**0.5

def main(args=None):
    rclpy.init(args=args)

    length = 2.0
    square_turtle = SquareTurtle(length)
    square_turtle.move_turtle()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
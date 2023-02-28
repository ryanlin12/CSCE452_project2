from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math
import rclpy


# turtlesim.msg.Pose.SharedPtr g_pose;
# turtlesim.msg.Pose g_goal
g_pose = None
g_gosl = None
enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

# class State:
  

State g_state = FORWARD
State g_last_state = FORWARD
g_first_goal_set = False

#define PI 3.141592

def poseCallback(pose):
  global g_pose
  g_pose = pose

def hasReachedGoal():
  return fabsf(g_pose->x - g_goal.x)+ < 0.1 and fabsf(g_pose->y - g_goal.y) < 0.1 and fabsf(g_pose->theta - g_goal.theta) < 0.01


def hasStopped():
  global g_pose
  return g_pose.angular_velocity < 0.0001 and g_pose.linear_velocity < 0.0001


def printGoal(): 
  global g_goal
  print("New goal [%f %f, %f]" % (g_goal.x, g_goal.y, g_goal.theta))


def commandTurtle(twist_pub, linear, angular):

  twist = Twist()
  twist.linear.x = linear
  twist.angular.z = angular
  twist_pub.publish(twist)


def stopForward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub):
  if (hasStopped())
  {
    RCLCPP_INFO(rclcpp::get_logger("draw_square"), "Reached goal");
    g_state = TURN;
    g_goal.x = g_pose->x;
    g_goal.y = g_pose->y;
    g_goal.theta = fmod(g_pose->theta + PI/2.0, 2*PI);
    // wrap g_goal.theta to [-pi, pi)
    if (g_goal.theta >= PI) g_goal.theta -= 2 * PI;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }


def stopTurn(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub):

  if (hasStopped())
  {
    RCLCPP_INFO(rclcpp::get_logger("draw_square"), "Reached goal");
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }



def forward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub):

  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 1.0, 0.0);
  }


def turn(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub):
  if hasReachedGoal():
    g_state = STOP_TURN
    commandTurtle(twist_pub, 0, 0)
  else:
    commandTurtle(twist_pub, 0.0, 0.4)


def timerCallback(twist_pub):

  if not g_pose:
    return
  

  if not g_first_goal_set:
  
    g_first_goal_set = True
    g_state = FORWARD
    g_goal.x = cos(g_pose->theta) * 2 + g_pose->x
    g_goal.y = sin(g_pose->theta) * 2 + g_pose->y
    g_goal.theta = g_pose->theta
    printGoal()
  

  if g_state == FORWARD:
    forward(twist_pub)
  
  elif g_state == STOP_FORWARD:
    stopForward(twist_pub);
  
  elif g_state == TURN:
    turn(twist_pub)
  
  elif g_state == STOP_TURN:
    stopTurn(twist_pub)


def main():
    rclpy.init(args=args)
    auto nh = rclcpp::Node::make_shared("draw_square");
    
    auto pose_sub = nh->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(poseCallback, std::placeholders::_1));
    #self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', 1000, self.pose_callback)
    #self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1000)
    
    auto twist_pub = nh->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    auto reset = nh->create_client<std_srvs::srv::Empty>("reset");
    auto timer = nh->create_wall_timer(std::chrono::milliseconds(16), [twist_pub](){timerCallback(twist_pub);});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset->async_send_request(empty);

    rclcpp::spin(nh);


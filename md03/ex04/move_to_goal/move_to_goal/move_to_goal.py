import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class MoveToGoal(Node):
    def __init__(self, x_goal, y_goal, theta_goal):
        super().__init__('move_to_goal')
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.theta_goal = theta_goal

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.move)
        self.reached = False

    def pose_callback(self, msg):
        self.pose = msg

    def move(self):
        if self.reached:
            return
        msg = Twist()
        distance = math.sqrt((self.x_goal - self.pose.x)**2 + (self.y_goal - self.pose.y)**2)
        angle_to_goal = math.atan2(self.y_goal - self.pose.y, self.x_goal - self.pose.x)
        angular_error = angle_to_goal - self.pose.theta

        msg.linear.x = 1.5 * distance
        msg.angular.z = 4.0 * angular_error

        if distance < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.reached = True
            self.get_logger().info('Goal reached!')
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal x y theta")
        return
    x, y, theta = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    node = MoveToGoal(x, y, theta)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import tf_transformations
from tf2_ros import TransformBroadcaster

class TurtleTFBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.sub2 = self.create_subscription(
            Pose, '/turtle2/pose', self.turtle2_callback, 10)

    def handle_pose(self, msg, name):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = name
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    def turtle1_callback(self, msg):
        self.handle_pose(msg, 'turtle1')

    def turtle2_callback(self, msg):
        self.handle_pose(msg, 'turtle2')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

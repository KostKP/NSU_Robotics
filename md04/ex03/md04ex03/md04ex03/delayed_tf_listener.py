import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
from rclpy.time import Time

class TurtleTF2Listener(Node):
    def __init__(self, delay_sec: float):
        super().__init__('turtle_tf2_listener')
        self.delay = delay_sec
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f"Delay set to {self.delay} seconds")

    def on_timer(self):
        try:
            now = self.get_clock().now()
            when = now - Duration(seconds=self.delay)

            trans = self.tf_buffer.lookup_transform_full(
                target_frame='turtle2',
                target_time=Time(),
                source_frame='turtle1',
                source_time=when,
                fixed_frame='world',
                timeout=Duration(seconds=0.05))

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('Transform not ready yet...')
            return

        msg = Twist()
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        msg.angular.z = 4.0 * math.atan2(y, x)
        msg.linear.x = 0.5 * math.sqrt(x * x + y * y)
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    import sys
    delay = 5.0
    if len(sys.argv) > 1:
        try:
            delay = float(sys.argv[1])
        except ValueError:
            pass
    node = TurtleTF2Listener(delay)
    rclpy.spin(node)
    rclpy.shutdown()

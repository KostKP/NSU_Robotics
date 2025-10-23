import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_tf2_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz

    def timer_cb(self):
        try:
            # хотим получить позицию carrot в системе координат turtle2
            now = rclpy.time.Time()  # пустое время = latest
            trans = self.tf_buffer.lookup_transform('turtle2', 'carrot', now)
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            dist = math.sqrt(dx*dx + dy*dy)
            angle_to_target = math.atan2(dy, dx)
            # формируем команду: линейная - по расстоянию, угловая - по углу
            cmd = Twist()
            cmd.linear.x = 1.5 * dist     # коэффициенты можно подбирать
            cmd.angular.z = 4.0 * angle_to_target
            self.publisher.publish(cmd)
        except TransformException:
            # трансформация пока недоступна - ничего не публикуем
            return

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

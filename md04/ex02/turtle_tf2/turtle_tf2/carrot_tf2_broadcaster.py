import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class CarrotTfBroadcaster(Node):
    def __init__(self):
        super().__init__('carrot_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        # параметры: radius и направление вращения
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('direction_of_rotation', 1)

        self.radius = float(self.get_parameter('radius').value)
        # приведём direction к int и ограничим значениями 1 или -1
        dir_val = int(self.get_parameter('direction_of_rotation').value)
        self.direction = 1 if dir_val >= 0 else -1

        self.angle = 0.0
        # частота вещания (например 20 Hz)
        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'   # фрейм, вокруг которого вращается морковка
        t.child_frame_id = 'carrot'
        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # нет вращения относительно своей оси
        self.tf_broadcaster.sendTransform(t)

        # увеличиваем угол; скорость угловая — регулируй, если надо
        self.angle += 0.08 * self.direction

def main(args=None):
    rclpy.init(args=args)
    node = CarrotTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

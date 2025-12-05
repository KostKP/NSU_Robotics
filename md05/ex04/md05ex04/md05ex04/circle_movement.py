import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.declare_parameter('linear_speed', 0.2)   # м/с
        self.declare_parameter('angular_speed', 0.5)  # рад/с
        self.declare_parameter('rate', 10.0)          # Гц

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.rate = self.get_parameter('rate').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Таймер для проверки clock
        self.clock_checker_timer = self.create_timer(0.1, self._check_clock)

        self.movement_timer = None
        self.get_logger().info('Circle mover initialized, waiting for /clock...')

    def _check_clock(self):
        # Ждём, пока ROS2 clock начнёт приходить
        if self.get_clock().now().nanoseconds > 0:
            # Создаём основной таймер для движения
            period = 1.0 / float(self.rate)
            self.movement_timer = self.create_timer(period, self.timer_callback)
            self.get_logger().info(f'Clock active — starting movement: linear={self.linear_speed}, angular={self.angular_speed}')
            # Удаляем таймер проверки clock
            self.clock_checker_timer.cancel()

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = float(self.linear_speed)
        twist.angular.z = float(self.angular_speed)
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

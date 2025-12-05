import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MultiMovementNode(Node):
    def __init__(self):
        super().__init__('multi_movement')
        
        # Параметры
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('spiral_radius', 2.0)
        self.declare_parameter('movement_radius', 3.0)  # Радиус для случайных точек
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.rate = self.get_parameter('rate').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.spiral_radius = self.get_parameter('spiral_radius').value
        self.movement_radius = self.get_parameter('movement_radius').value
        
        # Состояния
        self.state = 'WAITING_FOR_POSE'  # WAITING_FOR_POSE, MOVE_TO_POINT, SPIRAL
        self.pose = None
        self.current_goal = None
        self.spiral_points = []
        self.current_spiral_index = 0
        self.spiral_center = None
        
        # Издатели и подписчики
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_cb, 10)
        
        # Таймер для проверки clock
        self.clock_checker_timer = self.create_timer(0.1, self._check_clock)
        
        # Основной таймер для управления движением
        self.movement_timer = None
        
        self.get_logger().info('Multi-movement node initialized, waiting for /clock and /odom...')

    def _check_clock(self):
        # Ждём, пока ROS2 clock начнёт приходить
        if self.get_clock().now().nanoseconds > 0 and self.pose is not None:
            # Создаём основной таймер для движения
            period = 1.0 / float(self.rate)
            self.movement_timer = self.create_timer(period, self.timer_callback)
            self.state = 'MOVE_TO_POINT'
            self._generate_random_goal()
            self.get_logger().info('Clock active — starting movement sequence')
            # Удаляем таймер проверки clock
            self.clock_checker_timer.cancel()

    def pose_cb(self, msg):
        # Сохраняем текущую позу
        self.pose = msg.pose.pose

    def _generate_random_goal(self):
        """Генерирует случайную точку в пределах movement_radius от текущего положения"""
        if self.pose is None:
            return
        
        # Текущая позиция
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        
        # Генерируем случайный угол и радиус
        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(1.0, self.movement_radius)
        
        # Вычисляем целевую точку
        goal_x = current_x + radius * math.cos(angle)
        goal_y = current_y + radius * math.sin(angle)
        
        self.current_goal = (goal_x, goal_y)
        self.get_logger().info(f'New random goal: ({goal_x:.2f}, {goal_y:.2f})')

    def _generate_spiral_points(self):
        """Генерирует точки для спирали Архимеда"""
        if self.pose is None:
            return []
        
        # Центр спирали - текущее положение
        center_x = self.pose.position.x
        center_y = self.pose.position.y
        self.spiral_center = (center_x, center_y)
        
        # Параметры спирали
        b = 0.08  # Коэффициент роста спирали (можно сделать параметром)
        theta_max = self.spiral_radius / b + 0.5
        num_points = max(50, int(theta_max * 8))
        
        waypoints = []
        for i in range(num_points + 1):
            theta = (i / num_points) * theta_max
            r = b * theta
            x = center_x + r * math.cos(theta)
            y = center_y + r * math.sin(theta)
            waypoints.append((x, y))
        
        self.get_logger().info(f'Generated spiral with {len(waypoints)} points')
        return waypoints

    def _normalize_angle(self, angle):
        """Нормализует угол в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _get_yaw_from_quaternion(self, quaternion):
        """Извлекает угол yaw из кватерниона"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Вычисляем yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def timer_callback(self):
        if self.pose is None:
            return
        
        twist = Twist()
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        
        if self.state == 'MOVE_TO_POINT':
            if self.current_goal is None:
                self._generate_random_goal()
                return
            
            goal_x, goal_y = self.current_goal
            
            # Вычисляем ошибку
            dx = goal_x - current_x
            dy = goal_y - current_y
            distance = math.hypot(dx, dy)
            
            # Получаем текущий угол
            current_yaw = self._get_yaw_from_quaternion(self.pose.orientation)
            
            # Вычисляем целевой угол
            target_angle = math.atan2(dy, dx)
            angle_error = self._normalize_angle(target_angle - current_yaw)
            
            # Пропорциональный контроллер
            linear_gain = 0.8
            angular_gain = 2.5
            
            twist.linear.x = linear_gain * distance
            twist.angular.z = angular_gain * angle_error
            
            # Ограничение скорости
            twist.linear.x = max(0.05, min(twist.linear.x, self.max_linear_speed))
            twist.angular.z = max(-self.max_angular_speed, 
                                 min(twist.angular.z, self.max_angular_speed))
            
            # Если достигли цели
            if distance < self.goal_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub.publish(twist)
                
                # Переходим к спирали
                self.state = 'SPIRAL'
                self.spiral_points = self._generate_spiral_points()
                self.current_spiral_index = 0
                self.get_logger().info('Reached goal! Starting spiral movement...')
                return
        
        elif self.state == 'SPIRAL':
            if not self.spiral_points:
                self.spiral_points = self._generate_spiral_points()
                self.current_spiral_index = 0
            
            if self.current_spiral_index >= len(self.spiral_points):
                # Спираль завершена, переходим к случайной точке
                self.state = 'MOVE_TO_POINT'
                self._generate_random_goal()
                self.get_logger().info('Spiral completed! Moving to new random point...')
                return
            
            # Текущая целевая точка спирали
            goal_x, goal_y = self.spiral_points[self.current_spiral_index]
            
            # Вычисляем ошибку
            dx = goal_x - current_x
            dy = goal_y - current_y
            distance = math.hypot(dx, dy)
            
            # Получаем текущий угол
            current_yaw = self._get_yaw_from_quaternion(self.pose.orientation)
            
            # Вычисляем целевой угол
            target_angle = math.atan2(dy, dx)
            angle_error = self._normalize_angle(target_angle - current_yaw)
            
            # Пропорциональный контроллер (более мягкий для спирали)
            linear_gain = 0.5
            angular_gain = 2.0
            
            twist.linear.x = linear_gain * distance
            twist.angular.z = angular_gain * angle_error
            
            # Ограничение скорости
            twist.linear.x = max(0.05, min(twist.linear.x, self.max_linear_speed * 0.7))
            twist.angular.z = max(-self.max_angular_speed * 0.7, 
                                 min(twist.angular.z, self.max_angular_speed * 0.7))
            
            # Если достигли текущей точки спирали
            if distance < self.goal_tolerance * 1.5:
                self.current_spiral_index += 1
        
        # Публикуем команду
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MultiMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Останавливаем робота при завершении
        stop_twist = Twist()
        node.pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

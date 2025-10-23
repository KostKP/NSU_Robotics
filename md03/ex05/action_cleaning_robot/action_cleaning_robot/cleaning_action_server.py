import math
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from action_cleaning_robot_interfaces.action import CleaningTask


class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = Pose()
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        # Action server
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'cleaning_action',
            self.execute_callback)

        self.get_logger().info('Cleaning Action Server started')

        # For graceful stopping of motion loops
        self._stop_event = Event()

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        task_type = goal.task_type

        self.get_logger().info(f'Received goal: {task_type} | area_size={goal.area_size} | target=({goal.target_x},{goal.target_y})')

        feedback_msg = CleaningTask.Feedback()
        result = CleaningTask.Result()

        cleaned_points = 0
        total_distance = 0.0

        # Helper: move to a single waypoint (x,y) with simple proportional controller
        def move_to_point(x_goal, y_goal, stop_on_cancel=True):
            nonlocal total_distance
            rate_hz = 20.0
            dt = 1.0 / rate_hz
            reached = False
            prev_x = self.pose.x
            prev_y = self.pose.y

            max_time = 60.0  # safety timeout per waypoint
            t0 = time.time()

            while rclpy.ok() and not reached:
                if stop_on_cancel and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return False

                # compute control
                dx = x_goal - self.pose.x
                dy = y_goal - self.pose.y
                distance = math.hypot(dx, dy)
                angle_to_goal = math.atan2(dy, dx)
                angular_error = self._normalize_angle(angle_to_goal - self.pose.theta)

                cmd = Twist()
                
                # Параметры спирали
                linear_gain = 5.0
                angular_gain = 12.0
                max_linear = 8.0
                max_angular = 16.0
                
                # proportional controllers
                cmd.linear.x = linear_gain * distance
                cmd.angular.z = angular_gain * angular_error

                if cmd.linear.x > max_linear:
                    cmd.linear.x = max_linear
                if cmd.angular.z > max_angular:
                    cmd.angular.z = max_angular
                if cmd.angular.z < -max_angular:
                    cmd.angular.z = -max_angular

                # if close enough
                if distance < 0.08:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    reached = True

                self.cmd_pub.publish(cmd)

                # integrate distance traveled approximately
                step_dist = math.hypot(self.pose.x - prev_x, self.pose.y - prev_y)
                total_distance += step_dist
                prev_x = self.pose.x
                prev_y = self.pose.y

                # publish feedback
                feedback_msg.current_cleaned_points = cleaned_points
                feedback_msg.current_x = self.pose.x
                feedback_msg.current_y = self.pose.y
                goal_handle.publish_feedback(feedback_msg)

                # timeout safety
                if time.time() - t0 > max_time:
                    self.get_logger().warn('Timeout moving to waypoint')
                    return False

                rclpy.spin_once(self, timeout_sec=dt)

            # stop after reaching
            self.cmd_pub.publish(Twist())
            return True

        # Execute tasks
        success = False
        try:
            if task_type == 'clean_circle':
                # Spiral cleaning around current pose
                start_x = self.pose.x
                start_y = self.pose.y
                radius = float(goal.area_size)

                if radius <= 0.0:
                    self.get_logger().warn('Requested radius <= 0, finishing')
                    result.success = False
                    result.cleaned_points = 0
                    result.total_distance = 0.0
                    goal_handle.abort()
                    return result

                # Generate spiral waypoints (Archimedean spiral r = b*theta)
                b = 0.03  # radial growth per radian (tweakable)
                theta_max = radius / b + 0.5
                num_points = max(40, int(theta_max * 6))

                waypoints = []
                for i in range(num_points + 1):
                    theta = (i / num_points) * theta_max
                    r = b * theta
                    x = start_x + r * math.cos(theta)
                    y = start_y + r * math.sin(theta)
                    waypoints.append((x, y))

                total_wp = len(waypoints)
                cleaned_points = 0

                for idx, (wx, wy) in enumerate(waypoints):
                    # cancellation check
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info('Goal canceled during cleaning')
                        result.success = False
                        result.cleaned_points = cleaned_points
                        result.total_distance = total_distance
                        return result

                    ok = move_to_point(wx, wy)
                    if not ok:
                        # failed to reach a waypoint
                        self.get_logger().warn('Failed to reach waypoint, aborting')
                        result.success = False
                        result.cleaned_points = cleaned_points
                        result.total_distance = total_distance
                        goal_handle.abort()
                        return result

                    cleaned_points += 1

                    # update feedback
                    progress = int((idx + 1) / total_wp * 100)
                    feedback_msg.progress_percent = progress
                    feedback_msg.current_cleaned_points = cleaned_points
                    feedback_msg.current_x = self.pose.x
                    feedback_msg.current_y = self.pose.y
                    goal_handle.publish_feedback(feedback_msg)

                success = True

            elif task_type == 'return_home':
                tx = float(goal.target_x)
                ty = float(goal.target_y)

                ok = move_to_point(tx, ty)
                if ok:
                    success = True
                    cleaned_points = 0
                else:
                    success = False

            else:
                self.get_logger().error(f'Unknown task_type: {task_type}')
                goal_handle.abort()
                result.success = False
                result.cleaned_points = 0
                result.total_distance = total_distance
                return result

            # finish
            if success:
                result.success = True
                result.cleaned_points = cleaned_points
                result.total_distance = total_distance
                goal_handle.succeed()
                self.get_logger().info('Goal succeeded')
                return result
            else:
                result.success = False
                result.cleaned_points = cleaned_points
                result.total_distance = total_distance
                goal_handle.abort()
                return result

        except Exception as e:
            self.get_logger().error('Exception in execute_callback: ' + str(e))
            result.success = False
            result.cleaned_points = cleaned_points
            result.total_distance = total_distance
            goal_handle.abort()
            return result

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CleaningActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

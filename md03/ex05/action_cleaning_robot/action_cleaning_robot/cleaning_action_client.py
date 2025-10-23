import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_cleaning_robot_interfaces.action import CleaningTask


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'cleaning_action')

    def send_goal_and_wait(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return None

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = float(area_size)
        goal_msg.target_x = float(target_x)
        goal_msg.target_y = float(target_y)

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info(f'Sent goal: {task_type}')

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        status = result_future.result().status

        self.get_logger().info(f'Result: success={result.success}, cleaned_points={result.cleaned_points}, total_distance={result.total_distance:.3f}')
        return result

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {fb.progress_percent}% | cleaned={fb.current_cleaned_points} | pos=({fb.current_x:.2f},{fb.current_y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    client = CleaningActionClient()

    # Example sequence: clean a 3x3 area (we interpret as radius=1.5 for circle), then return home to (5.5, 5.5)
    try:
        client.get_logger().info('Starting cleaning sequence from client')

        # Clean circle of radius 1.5 m (approx covering ~3x3 area)
        res1 = client.send_goal_and_wait('clean_circle', area_size=1.5)
        if res1 is None or not res1.success:
            client.get_logger().warn('Cleaning failed or was canceled')
        else:
            client.get_logger().info('Cleaning finished successfully')

        # Return home
        res2 = client.send_goal_and_wait('return_home', target_x=2.5, target_y=2.5)
        if res2 is None or not res2.success:
            client.get_logger().warn('Return home failed or was canceled')
        else:
            client.get_logger().info('Returned home successfully')

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

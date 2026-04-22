#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
from task2.action import Nav2Pose

class ManhattanClient(Node):
    def __init__(self):
        super().__init__('manhattan_client')
        
        self._client = ActionClient(self, Nav2Pose, 'navigate_to_pose')

    def send_goal(self, x, y):
        self.get_logger().info('Waiting for Action Server...')
        self._client.wait_for_server()

        goal_msg = Nav2Pose.Goal()
        
        goal_msg.target_x = float(x)
        goal_msg.target_y = float(y)

        self.get_logger().info(f'Sending goal: X={x}, Y={y}')

        self._send_goal_future = self._client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the server!')
            return

        self.get_logger().info('Goal accepted! Robot is moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """This function runs every time the server sends a progress update."""
        fb = feedback_msg.feedback
        
        self.get_logger().info(
            f'Phase: [{fb.phase}] | Pos: ({fb.current_x:.2f}, {fb.current_y:.2f}) | '
            f'Remaining: {fb.dist_remaining:.2f}m',
            throttle_duration_sec=0.5) 

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Mission accomplished! Final Position: ({result.final_x:.2f}, {result.final_y:.2f})')
        else:
            self.get_logger().error('Mission failed.')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Check if user gave coordinates in the terminal
    if len(sys.argv) < 3:
        print("Usage: ros2 run task2 action_client <x> <y>")
        return

    client = ManhattanClient()
    client.send_goal(sys.argv[1], sys.argv[2])

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
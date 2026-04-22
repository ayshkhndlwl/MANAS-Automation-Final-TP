#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from task2.action import Nav2Pose

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)

def manhattan_dist(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.cb_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            SENSOR_QOS, callback_group=self.cb_group)

        self.pose_sub = self.create_subscription(
            Pose2D, '/estimated_pose', self.pose_callback,
            10, callback_group=self.cb_group)

        self._action_server = ActionServer(
            self, Nav2Pose, 'navigate_to_pose',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        self.state = 'PLAN'
        self.next_state_after_turn = 'PLAN' 
        
        self.target_heading = 0.0
        self.seek_axis = 'X'
        self.seek_target = 0.0
        
        self.hit_point_dist = float('inf')
        self.safety_distance = 0.6
        self.clearance_start_x = 0.0
        self.clearance_start_y = 0.0
        
        self.front_dist = float('inf')
        self.right_dist = float('inf')

        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0

    def pose_callback(self, msg):
        self.current_x     = msg.x
        self.current_y     = msg.y
        self.current_theta = msg.theta

    def scan_callback(self, msg):
        if not msg.ranges: 
            return
        total = len(msg.ranges) 
        mid = total // 2        
        right_idx = total // 4  
        
        front_slice = [r for r in msg.ranges[mid-15 : mid+15] if math.isfinite(r) and r > msg.range_min]
        right_slice = [r for r in msg.ranges[right_idx-10 : right_idx+10] if math.isfinite(r) and r > msg.range_min]

        self.front_dist = min(front_slice) if front_slice else float('inf')
        self.right_dist = min(right_slice) if right_slice else float('inf')

    def stop_robot(self):
        self.publisher.publish(Twist())
        
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT    

    # --- STATE 1: PLAN CARDINAL MOVEMENT ---
    def logic_plan(self, target_x, target_y, fb):
        cmd = Twist() 
        
        if abs(target_x - self.current_x) > 0.05:
            self.seek_axis = 'X'
            self.seek_target = target_x
            self.target_heading = 0.0 if target_x > self.current_x else math.pi
        else:
            self.seek_axis = 'Y'
            self.seek_target = target_y
            self.target_heading = math.pi / 2.0 if target_y > self.current_y else -math.pi / 2.0

        self.state = 'TURN'
        self.next_state_after_turn = 'DRIVE'
        fb.phase = f'Planning {self.seek_axis}-axis move'
        return cmd

    # --- STATE 2: ROTATION ---
    def logic_turn(self, fb):
        cmd = Twist()
        angle_error = normalize_angle(self.target_heading -  self.current_theta)

        if abs(angle_error) > 0.05: 
            cmd.angular.z = 0.5 if angle_error > 0 else -0.5
            fb.phase = 'Turning to Cardinal Heading'
        else:
            self.stop_robot()
            self.state = self.next_state_after_turn
            if self.state == 'CATCH_WALL':
                self.catch_start_x = self.current_x
                self.catch_start_y = self.current_y
            
        return cmd

    # --- STATE 3: DRIVE STRAIGHT ON AXIS ---
    def logic_drive(self, target_x, target_y, fb):
        cmd = Twist()
        
        if self.front_dist < self.safety_distance:
            self.stop_robot()
            self.hit_point_dist = manhattan_dist(target_x, target_y, self.current_x, self.current_y)

            self.target_heading = normalize_angle(self.target_heading + (math.pi / 2.0))
            self.state = 'TURN'
            self.next_state_after_turn = 'WALL_FOLLOW'
            fb.phase = 'Obstacle Detected! Turning Left.'
            return cmd

        current_val = self.current_x if self.seek_axis == 'X' else self.current_y
        if abs(self.seek_target - current_val) < 0.05:
            self.stop_robot()
            self.state = 'PLAN'
            return cmd

        angle_error = normalize_angle(self.target_heading - self.current_theta)
        cmd.linear.x = 0.2
        cmd.angular.z = 0.5 * angle_error if abs(angle_error) > 0.05 else 0.0
        fb.phase = f'Driving along {self.seek_axis}-axis'
        return cmd

    # --- STATE 4: WALL FOLLOW ---
    def logic_wall_follow(self, target_x, target_y, fb):
        cmd = Twist()
        
        # 1. Escape Check
        current_dist = manhattan_dist(target_x, target_y, self.current_x, self.current_y)
        if current_dist < (self.hit_point_dist - 0.2):
            self.stop_robot()
            self.state = 'PLAN'
            fb.phase = 'Escaped Obstacle!'
            return cmd

        # 2. Inside Corner Check
        if self.front_dist < self.safety_distance:
            self.stop_robot()
            self.target_heading = normalize_angle(self.target_heading + (math.pi / 2.0))
            self.state = 'TURN'
            self.next_state_after_turn = 'WALL_FOLLOW'
            return cmd
            
        # 3. Outside Corner Check
        if self.right_dist > 0.9: 
            self.clearance_start_x = self.current_x
            self.clearance_start_y = self.current_y
            self.state = 'CLEAR_CORNER'
            return cmd

        # Keep the robot perfectly parallel to the wall
        angle_error = normalize_angle(self.target_heading - self.current_theta)
        cmd.linear.x = 0.3
        cmd.angular.z = 0.5 * angle_error if abs(angle_error) > 0.05 else 0.0
        
        fb.phase = 'Tracing Wall Segment'
        return cmd

    # --- STATE 5: CLEAR OUTSIDE CORNER ---
    def logic_clear_corner(self, fb):
        cmd = Twist()
        dist_moved = math.hypot(self.current_x - self.clearance_start_x, self.current_y - self.clearance_start_y)
        
        # If a wall appears in front while we are trying to clear the corner, abort and turn left.
        if self.front_dist < self.safety_distance:
            self.stop_robot()
            self.target_heading = normalize_angle(self.target_heading + (math.pi / 2.0))
            self.state = 'TURN'
            self.next_state_after_turn = 'WALL_FOLLOW'
            fb.phase = 'Emergency: Front Wall Detected during Clearance!'
            return cmd

        if dist_moved > 0.4: 
            self.stop_robot()
            self.target_heading = normalize_angle(self.target_heading - (math.pi / 2.0))
            self.state = 'TURN'
            self.next_state_after_turn = 'CATCH_WALL' 
        else:
            # Proportional Heading Lock to stay perfectly straight
            angle_error = normalize_angle(self.target_heading - self.current_theta)
            cmd.linear.x = 0.2
            cmd.angular.z = 0.5 * angle_error if abs(angle_error) > 0.05 else 0.0
            
            fb.phase = 'Clearing Outside Corner'
            
        return cmd
    
    # --- STATE 6: CATCH WALL AFTER CORNER ---
    def logic_catch_wall(self, target_x, target_y, fb):
        cmd = Twist()
        
        # 1. Bug 2 Escape Check (Always active)
        current_dist = manhattan_dist(target_x, target_y, self.current_x, self.current_y)
        if current_dist < (self.hit_point_dist - 0.2):
            self.stop_robot()
            self.state = 'PLAN'
            fb.phase = 'Escaped Obstacle!'
            return cmd

        # 2. Safety Net: Front Wall detected before we caught the right wall
        if self.front_dist < self.safety_distance:
            self.stop_robot()
            self.target_heading = normalize_angle(self.target_heading + (math.pi / 2.0))
            self.state = 'TURN'
            self.next_state_after_turn = 'WALL_FOLLOW'
            return cmd
            
        # 3. SUCCESS: The right sensor caught the wall!
        if self.right_dist < 0.9: 
            self.stop_robot()
            self.state = 'WALL_FOLLOW'
            fb.phase = 'Wall Caught!'
            return cmd
            
        # 4. SAFETY NET: Drove too far without catching a wall. Must be a thin post. Wrap again!
        dist_moved = math.hypot(self.current_x - self.catch_start_x, self.current_y - self.catch_start_y)
        if dist_moved > 0.8:
            self.stop_robot()
            self.clearance_start_x = self.current_x
            self.clearance_start_y = self.current_y
            self.state = 'CLEAR_CORNER'
            return cmd
            
        # Keep driving straight on the invisible rail until the sensor catches
        angle_error = normalize_angle(self.target_heading - self.current_theta)
        cmd.linear.x = 0.2
        cmd.angular.z = 0.5 * angle_error if abs(angle_error) > 0.05 else 0.0
        fb.phase = 'Catching New Wall Face'
        return cmd

    def execute_callback(self, goal_handle):
        tx = goal_handle.request.target_x
        ty = goal_handle.request.target_y
        fb = Nav2Pose.Feedback()
        cmd = Twist()

        self.state = 'PLAN'

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return Nav2Pose.Result()

            dist_to_goal = math.hypot(tx - self.current_x, ty - self.current_y)
            if dist_to_goal < 0.05:
                break 

            if self.state == 'PLAN':          cmd = self.logic_plan(tx, ty, fb)
            elif self.state == 'TURN':        cmd = self.logic_turn(fb)
            elif self.state == 'DRIVE':       cmd = self.logic_drive(tx, ty, fb)
            elif self.state == 'WALL_FOLLOW': cmd = self.logic_wall_follow(tx, ty, fb)
            elif self.state == 'CLEAR_CORNER':cmd = self.logic_clear_corner(fb)
            elif self.state == 'CATCH_WALL':  cmd = self.logic_catch_wall(tx, ty, fb) # <--- NEW
            
            try:
                fb.dist_remaining = dist_to_goal
                fb.current_x = self.current_x
                fb.current_y = self.current_y
            except AttributeError:
                pass 

            self.publisher.publish(cmd)
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

        self.stop_robot()
        goal_handle.succeed()
        result = Nav2Pose.Result()
        result.final_x = self.current_x
        result.final_y = self.current_y
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControllerNode(), executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
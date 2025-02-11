#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_action_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        
        # Joint names from your URDF
        goal_msg.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]
        
        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions  # Pass the list of positions
        point.time_from_start = Duration(sec=2)
        
        goal_msg.trajectory.points = [point]
        
        # Wait for action server
        self._action_client.wait_for_server()
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal completed')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback')

def main(args=None):
    rclpy.init(args=args)
    action_client = JointTrajectoryActionClient()
    
    # Send a goal to move to the specified positions
    joint_positions = [1.57, 1.57, 0.0, 0.0, 0.0, 0.0]  # Example positions
    action_client.send_goal(joint_positions)
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

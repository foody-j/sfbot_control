#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

    def send_trajectory(self, trajectory, joint_names):
        """
        JointTrajectory 메시지를 생성하고 액션 서버에 보냅니다.

        Args:
            trajectory (numpy.ndarray): 궤적 (각 행은 조인트 각도를 나타냄).
            joint_names (list): 조인트 이름 목록.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        
        # 각 점을 JointTrajectoryPoint로 변환
        points = []
        for i, angles in enumerate(trajectory):
            point = JointTrajectoryPoint()
            point.positions = angles.tolist() # NumPy 배열을 리스트로 변환
            point.time_from_start = Duration(sec=i, nanosec=0)  # 각 점의 시간 간격 설정 (예: 1초)
            points.append(point)
            
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal succeeded: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedback: {0}'.format(feedback))

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()

    # 예시 궤적 생성 (조인트 공간)
    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    end_angles = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    num_points = 10
    trajectory = np.linspace(start_angles, end_angles, num_points)

    # 액션 서버에 궤적 전송
    action_client.send_trajectory(trajectory, joint_names)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

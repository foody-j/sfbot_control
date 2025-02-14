#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class EndEffectorCircleController(Node):
    def __init__(self):
        super().__init__('end_effector_circle_controller')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]

        # 원 궤적 설정 (조인트 6 끝단 기준)
        self.tcp_x_center = 0.0  # TCP 기준 원 중심 X
        self.tcp_y_center = 0.3  # TCP 기준 원 중심 Y
        self.tcp_z_fixed = 0.2   # TCP 기준 원 높이 (Z 고정)
        self.radius = 0.05       # 원 반지름 (5cm)
        self.speed = 1.0         # 원을 도는 속도 (라디안/초)

        self.timer = self.create_timer(0.1, self.update_trajectory)  # 0.1초마다 갱신
        self.time_elapsed = 0.0

    def inverse_kinematics(self, x, y, z):
        """
        주어진 (X, Y, Z) 좌표를 TCP가 따라가도록 6축 로봇팔 역기구학 계산
        """
        l1, l2, l3, l4, l5, l6 = 0.07, 0.07, 0.07, 0.07, 0.07, 0.07  # 각 링크 길이 (예제 값)

        # 베이스에서 TCP까지 거리 계산
        theta1 = math.atan2(y, x)

        # 거리 기반 계산
        d = math.sqrt(x**2 + y**2)
        phi = math.atan2(z - self.tcp_z_fixed, d)

        # 삼각형 법칙 이용
        cos_theta2 = (l2**2 + l3**2 - d**2) / (2 * l2 * l3)
        theta2 = math.acos(cos_theta2) if abs(cos_theta2) <= 1 else 0.0
        theta3 = math.pi - theta2  # 보정

        # TCP 회전을 고려하여 조인트 4, 5, 6 설정
        theta4 = -theta1
        theta5 = phi
        theta6 = theta1 + math.pi / 2

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def update_trajectory(self):
        """
        엔드 이펙터(TCP)가 원을 그리도록 궤적을 생성하고, 조인트 명령을 전송
        """
        self.time_elapsed += 0.1  # 0.1초마다 갱신

        # 원 궤적 계산 (조인트 6 끝단 기준)
        theta = self.speed * self.time_elapsed  # 시간에 따른 각도 변화
        x = self.tcp_x_center + self.radius * math.cos(theta)
        y = self.tcp_y_center + self.radius * math.sin(theta)
        z = self.tcp_z_fixed  # 고정된 높이 유지

        # 역기구학 계산 (X, Y, Z → 조인트 각도)
        joint_positions = self.inverse_kinematics(x, y, z)

        # ROS2 액션 클라이언트를 통해 조인트 명령 전송
        self.send_goal(joint_positions)

    def send_goal(self, joint_positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5초 후 도착

        goal_msg.trajectory.points = [point]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
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

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorCircleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

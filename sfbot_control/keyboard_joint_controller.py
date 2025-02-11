#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import select
import termios
import tty

class KeyboardJointController(Node):
    def __init__(self):
        super().__init__('keyboard_joint_controller')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 초기 위치

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.settings)
        return key

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds

        goal_msg.trajectory.points = [point]

        self._action_client.wait_for_server()
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
        # self.get_logger().info('Received feedback')

    def run(self):
        print("Controlling joints with keyboard:")
        print("---------------------------")
        print("joint 1: W/S")
        print("joint 2: E/D")
        print("joint 3: R/F")
        print("joint 4: T/G")
        print("joint 5: Y/H")
        print("joint 6: U/J")
        print("---------------------------")
        print("Press CTRL-C to quit")

        while rclpy.ok():
            key = self.getKey()
            if key == 'w':
                self.joint_positions[0] += 10
            elif key == 's':
                self.joint_positions[0] -= 10
            elif key == 'e':
                self.joint_positions[1] += 10
            elif key == 'd':
                self.joint_positions[1] -= 10
            elif key == 'r':
                self.joint_positions[2] += 10
            elif key == 'f':
                self.joint_positions[2] -= 10
            elif key == 't':
                self.joint_positions[3] += 10
            elif key == 'g':
                self.joint_positions[3] -= 10
            elif key == 'y':
                self.joint_positions[4] += 10
            elif key == 'h':
                self.joint_positions[4] -= 10
            elif key == 'u':
                self.joint_positions[5] += 10
            elif key == 'j':
                self.joint_positions[5] -= 10
            elif key == '\x03':  # CTRL-C
                break

            self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    keyboard_joint_controller = KeyboardJointController()
    keyboard_joint_controller.run()
    keyboard_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

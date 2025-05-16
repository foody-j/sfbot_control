#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
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

        # 초기 관절 위치와 증분값 설정
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.small_increment = 0.314
        self.large_increment = 3.14
        self.current_increment = self.small_increment
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Action 서버가 준비될 때까지 대기
        self._action_client.wait_for_server()

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
        # 액션 목표 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]
        
        # 단일 트라젝토리 포인트 생성
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = Duration(sec=2)
        goal_msg.trajectory.points = [point]
        
        # 목표 전송
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

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
        print("Q: Toggle increment (0.314/3.14)")
        print("O: Reset all positions to 0.0")
        print("Press CTRL-C to quit")

        while rclpy.ok():
            key = self.getKey()
            movement_detected = False

            if key == 'w':
                self.joint_positions[0] += self.current_increment
                movement_detected = True
            elif key == 's':
                self.joint_positions[0] -= self.current_increment
                movement_detected = True
            elif key == 'e':
                self.joint_positions[1] += self.current_increment
                movement_detected = True
            elif key == 'd':
                self.joint_positions[1] -= self.current_increment
                movement_detected = True
            elif key == 'r':
                self.joint_positions[2] += self.current_increment
                movement_detected = True
            elif key == 'f':
                self.joint_positions[2] -= self.current_increment
                movement_detected = True
            elif key == 't':
                self.joint_positions[3] += self.current_increment
                movement_detected = True
            elif key == 'g':
                self.joint_positions[3] -= self.current_increment
                movement_detected = True
            elif key == 'y':
                self.joint_positions[4] += self.current_increment
                movement_detected = True
            elif key == 'h':
                self.joint_positions[4] -= self.current_increment
                movement_detected = True
            elif key == 'u':
                self.joint_positions[5] += self.current_increment
                movement_detected = True
            elif key == 'j':
                self.joint_positions[5] -= self.current_increment
                movement_detected = True
            elif key == 'z':
                try:
                    print("Enter 6 joint positions separated by spaces (e.g., 0.0 0.0 0.0 0.0 0.0 0.0):")
                    input_positions = input()
                    positions = list(map(float, input_positions.split()))
                    if len(positions) != 6:
                        print("Error: Please enter exactly 6 values.")
                        continue
                    self.joint_positions = positions
                    movement_detected = True
                except ValueError:
                    print("Error: Invalid input. Please enter numeric values.")
            elif key == 'q':
                self.current_increment = self.large_increment if self.current_increment == self.small_increment else self.small_increment
                print(f"Increment set to {self.current_increment}")
            elif key == 'o':
                self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                movement_detected = True
            elif key == '\x03':  # CTRL-C
                break

            if movement_detected:
                print(f"Moving to positions: {self.joint_positions}")
                self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardJointController()
    try:
        controller.run()
    except Exception as e:
        print(e)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

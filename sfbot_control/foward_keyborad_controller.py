#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty

class KeyboardJointController(Node):
    def __init__(self):
        super().__init__('keyboard_joint_controller')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 초기 위치
        self.small_increment = 0.314
        self.large_increment = 3.14
        self.current_increment = self.small_increment
        self.large_increment_mode = False

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

    def publish_positions(self):
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)

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
        print("Q: Toggle increment between 0.314 and 3.14")
        print("O: Reset all joint positions to 0.0")
        print("Press CTRL-C to quit")

        while rclpy.ok():
            key = self.getKey()
            if key == 'w':
                self.joint_positions[0] += self.current_increment
            elif key == 's':
                self.joint_positions[0] -= self.current_increment
            elif key == 'e':
                self.joint_positions[1] += self.current_increment
            elif key == 'd':
                self.joint_positions[1] -= self.current_increment
            elif key == 'r':
                self.joint_positions[2] += self.current_increment
            elif key == 'f':
                self.joint_positions[2] -= self.current_increment
            elif key == 't':
                self.joint_positions[3] += self.current_increment
            elif key == 'g':
                self.joint_positions[3] -= self.current_increment
            elif key == 'y':
                self.joint_positions[4] += self.current_increment
            elif key == 'h':
                self.joint_positions[4] -= self.current_increment
            elif key == 'u':
                self.joint_positions[5] += self.current_increment
            elif key == 'j':
                self.joint_positions[5] -= self.current_increment
            elif key == 'q':
                if self.large_increment_mode:
                    self.current_increment = self.small_increment
                    self.large_increment_mode = False
                    print("Increment set to 0.314")
                else:
                    self.current_increment = self.large_increment
                    self.large_increment_mode = True
                    print("Increment set to 3.14")
            elif key == 'o':
                self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                print("All joint positions reset to 0.0")
            elif key == '\x03':  # CTRL-C
                break

            self.publish_positions()
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    keyboard_joint_controller = KeyboardJointController()
    keyboard_joint_controller.run()
    keyboard_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
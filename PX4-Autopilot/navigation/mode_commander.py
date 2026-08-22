#!/usr/bin/env python3
"""
follow_waypoints.py와 별도의 터미널에서 실행하는 명령어 입력 창.

follow_waypoints.py는 로그만 계속 출력하고,
여기서 mode1 / mode2 / auto 를 입력하면 /mode_command 토픽으로 전송된다.

사용법:
    python3 mode_commander.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeCommander(Node):
    def __init__(self):
        super().__init__('mode_commander')
        self.pub = self.create_publisher(String, '/mode_command', 10)

    def send(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ModeCommander()

    print('=== 명령어 입력 창 ===')
    print('mode1 / mode2 / auto 입력 (Ctrl+C로 종료)')

    try:
        while rclpy.ok():
            cmd = input('> ').strip().lower()
            if not cmd:
                continue
            node.send(cmd)
            print(f'-> 전송됨: {cmd}')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

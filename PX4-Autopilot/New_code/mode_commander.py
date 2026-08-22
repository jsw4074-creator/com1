import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MODE_COMMAND_TOPIC = '/mode_command'

VALID_COMMANDS = {
    'm1',
    'm2',
    'm3',
    'stop',
    'idle',
    'land',
    'debug_on',
    'debug_off',
}


class ModeCommander(Node):
    def __init__(self):
        super().__init__('mode_commander')

        self.publisher = self.create_publisher(
            String,
            MODE_COMMAND_TOPIC,
            10,
        )

    def send_command(self, command):
        command = command.strip().lower()

        if command not in VALID_COMMANDS:
            raise ValueError(
                '명령은 m1, m2, m3, stop, idle, land, '
                'debug_on, debug_off 중 하나여야 합니다.'
            )

        msg = String()
        msg.data = command

        self.publisher.publish(msg)

        self.get_logger().info(
            f'명령 전송: {command}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ModeCommander()

    try:
        print('사용 가능한 명령')
        print('  m1        : MODE1 시작')
        print('  m2        : MODE2 시작')
        print('  m3        : MODE3 시작')
        print('  stop      : 미션 정지')
        print('  idle      : IDLE 전환')
        print('  land      : 착륙')
        print('  debug_on  : 디버그 화면 켜기')
        print('  debug_off : 디버그 화면 끄기')
        print('  exit      : Commander 종료')

        while rclpy.ok():
            command = input('command> ').strip().lower()

            if command in ('exit', 'quit'):
                break

            if not command:
                continue

            try:
                node.send_command(command)

                rclpy.spin_once(
                    node,
                    timeout_sec=0.1,
                )

            except ValueError as exc:
                print(exc)

    except (KeyboardInterrupt, EOFError):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

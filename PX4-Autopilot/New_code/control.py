from coordinate import euler_to_quaternion
from config import (
    ATTITUDE_SETPOINT_TOPIC,
    CUSTOM_MAIN_MODE,
    MAX_THRUST,
    MIN_THRUST,
    OFFBOARD_MODE,
    OFFBOARD_MODE_TOPIC,
    POSITION_SETPOINT_TOPIC,
    SOURCE_COMPONENT,
    SOURCE_SYSTEM,
    TARGET_COMPONENT,
    TARGET_SYSTEM,
    VEHICLE_COMMAND_TOPIC,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    VehicleCommand,
)
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


# main.py에서 전달받은 ROS 2 노드
_node = None

# PX4 명령 퍼블리셔
_offboard_mode_publisher = None
_position_setpoint_publisher = None
_attitude_setpoint_publisher = None
_vehicle_command_publisher = None

def _timestamp_us():
    """
    현재 ROS 시간을 마이크로초 단위로 반환한다.
    """
    if _node is None:
        return 0

    return int(
        _node.get_clock().now().nanoseconds / 1000
    )


def _publish_offboard_mode(
    position=False,
    attitude=False,
):
    """
    PX4에 현재 사용할 오프보드 제어 입력 종류를 전송한다.

    Args:
        position:
            위치 목표 사용 여부

        attitude:
            자세 목표 사용 여부

    Returns:
        True:
            전송 성공

        False:
            초기화되지 않음
    """
    if _offboard_mode_publisher is None:
        return False

    msg = OffboardControlMode()

    msg.timestamp = _timestamp_us()

    msg.position = bool(position)
    msg.velocity = False
    msg.acceleration = False
    msg.attitude = bool(attitude)
    msg.body_rate = False

    _offboard_mode_publisher.publish(msg)

    return True

def _publish_vehicle_command(
    command,
    param1=0.0,
    param2=0.0,
    param3=0.0,
    param4=0.0,
    param5=0.0,
    param6=0.0,
    param7=0.0,
):
    """
    PX4에 VehicleCommand 메시지를 전송한다.

    Args:
        command:
            VehicleCommand 명령 번호

        param1 ~ param7:
            명령별 파라미터

    Returns:
        True:
            전송 성공

        False:
            초기화되지 않음
    """
    if _vehicle_command_publisher is None:
        return False

    msg = VehicleCommand()

    msg.timestamp = _timestamp_us()

    msg.command = int(command)

    msg.param1 = float(param1)
    msg.param2 = float(param2)
    msg.param3 = float(param3)
    msg.param4 = float(param4)
    msg.param5 = float(param5)
    msg.param6 = float(param6)
    msg.param7 = float(param7)

    msg.target_system = TARGET_SYSTEM
    msg.target_component = TARGET_COMPONENT

    msg.source_system = SOURCE_SYSTEM
    msg.source_component = SOURCE_COMPONENT

    msg.from_external = True

    _vehicle_command_publisher.publish(msg)

    return True


def init_cmd(node):
    """
    PX4 오프보드 제어에 필요한 퍼블리셔를 초기화한다.

    Args:
        node:
            main.py에서 생성한 ROS 2 Node
    """
    global _node
    global _offboard_mode_publisher
    global _position_setpoint_publisher
    global _attitude_setpoint_publisher
    global _vehicle_command_publisher

    _node = node

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    _offboard_mode_publisher = node.create_publisher(
        OffboardControlMode,
        OFFBOARD_MODE_TOPIC,
        qos,
    )

    _position_setpoint_publisher = node.create_publisher(
        TrajectorySetpoint,
        POSITION_SETPOINT_TOPIC,
        qos,
    )

    _attitude_setpoint_publisher = node.create_publisher(
        VehicleAttitudeSetpoint,
        ATTITUDE_SETPOINT_TOPIC,
        qos,
    )

    _vehicle_command_publisher = node.create_publisher(
        VehicleCommand,
        VEHICLE_COMMAND_TOPIC,
        qos,
    )


def cmd_att_offboard(
    roll,
    pitch,
    yaw,
    thrust,
):
    """
    자세제어 오프보드 명령을 PX4에 전송한다.

    이 함수는 자세제어 중 제어 주기마다 반복 호출한다.

    Args:
        roll, pitch, yaw:
            목표 자세각 [rad]

        thrust:
            정규화된 추력 명령

    Returns:
        True:
            전송 성공

        False:
            초기화되지 않음
    """
    if _attitude_setpoint_publisher is None:
        return False

    # 추력 명령 범위 제한
    thrust = max(
        MIN_THRUST,
        min(MAX_THRUST, float(thrust)),
    )

    # 자세제어 오프보드 입력 사용
    if not _publish_offboard_mode(
        position=False,
        attitude=True,
    ):
        return False

    msg = VehicleAttitudeSetpoint()

    msg.timestamp = _timestamp_us()

    # Euler 각도를 목표 quaternion으로 변환
    msg.q_d = [
        float(value)
        for value in euler_to_quaternion(
            float(roll),
            float(pitch),
            float(yaw),
        )
    ]

    # Yaw 각속도 피드포워드 미사용
    msg.yaw_sp_move_rate = 0.0

    # PX4 body FRD 좌표계에서 상승 추력은 -Z 방향
    msg.thrust_body = [
        0.0,
        0.0,
        -thrust,
    ]

    _attitude_setpoint_publisher.publish(msg)

    return True


def cmd_pos_offboard(
    x_hat,
    y_hat,
    z_hat,
    err_x,
    err_y,
    err_z,
    yaw,
):
    """
    현재 추정 위치와 위치 보정량으로 목표 위치를 생성해 전송한다.

    이 함수는 위치제어 중 제어 주기마다 반복 호출한다.

    Args:
        x_hat, y_hat, z_hat:
            현재 추정 위치 [m]
            PX4 NED 좌표계

        err_x, err_y, err_z:
            현재 위치에 더할 보정량 [m]
            PX4 NED 좌표계

        yaw:
            목표 Yaw [rad]

    Returns:
        True:
            전송 성공

        False:
            초기화되지 않음
    """
    if _position_setpoint_publisher is None:
        return False

    # 현재 위치와 보정량으로 목표 위치 계산
    x_setpoint = float(x_hat + err_x)
    y_setpoint = float(y_hat + err_y)
    z_setpoint = float(z_hat + err_z)

    # 위치제어 오프보드 입력 사용
    if not _publish_offboard_mode(
        position=True,
        attitude=False,
    ):
        return False

    msg = TrajectorySetpoint()

    msg.timestamp = _timestamp_us()

    msg.position = [
        x_setpoint,
        y_setpoint,
        z_setpoint,
    ]

    # 위치 명령 외의 입력은 사용하지 않음
    msg.velocity = [float('nan')] * 3
    msg.acceleration = [float('nan')] * 3
    msg.jerk = [float('nan')] * 3

    msg.yaw = float(yaw)
    msg.yawspeed = float('nan')

    _position_setpoint_publisher.publish(msg)

    return True

def set_offboard():
    """
    PX4 비행 모드를 Offboard로 전환한다.

    OffboardControlMode와 Setpoint를 먼저 일정 시간 전송한 뒤 호출한다.
    """
    return _publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        param1=CUSTOM_MAIN_MODE,
        param2=OFFBOARD_MODE,
    )


def arm():
    """
    기체에 ARM 명령을 전송한다.
    """
    return _publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        param1=1.0,
    )


def disarm():
    """
    기체에 DISARM 명령을 전송한다.
    """
    return _publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        param1=0.0,
    )
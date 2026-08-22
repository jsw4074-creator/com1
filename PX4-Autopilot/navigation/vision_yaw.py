"""
vision_yaw.py

라인의 두 지점(상/하 또는 좌/우) 좌표로부터, 카메라 프레임 축을 기준으로
라인이 몇 도 기울어져 보이는지(yaw)를 계산하는 모듈.

주의:
- 이 값은 위치(offset, dy)가 아니라 순수 "기울기(회전각)"만 나타냄.
- 드론의 롤/피치 보정이 안 된 순수 시각 기반 근사값 - odom yaw와
  대략적으로 비교하는 용도로만 사용할 것.

수정 이력(중요):
- compute_horizontal_yaw에 음수 반전을 넣어 compute_vertical_yaw과 부호를
  맞추는 시도를 재적용한다 (2번째 시도).
- 1차 시도 때는 첫 인식 라인이 'hor'인 상태로 부팅(EKF2_MAG_TYPE=None이라
  최초 yaw 정렬을 EV yaw 하나에만 의존)하면서 이륙 실패가 재현됐는데,
  이게 부호 반전 자체의 문제인지, 'hor' 첫부팅 시 초기 정렬 타이밍 문제가
  겹친 것인지 분리가 안 된 상태로 되돌렸었다.
- 이번 재검증은 반드시 'ver' 라인 위에서 부팅해서 초기 yaw 정렬을
  vertical_yaw로 먼저 마친 뒤, 이후 'hor' 전환 시 불연속/이륙 문제가
  재현되는지만 순수하게 확인하는 것이 목적. 만약 이번에도 실패하면,
  부호 반전 자체가 틀렸다는 뜻이므로 원복 + compute_vertical_yaw 쪽에
  반전을 넣는 방향으로 전환할 것.
"""

import math


def compute_vertical_yaw(l_top, r_top, y_top, l_bot, r_bot, y_bot):
    """
    V 모드(수직 라인)용 yaw 계산.

    Args:
        l_top, r_top: 상단 밴드에서 검출된 라인의 좌/우 x좌표
        y_top: 상단 밴드의 y좌표
        l_bot, r_bot: 하단 밴드에서 검출된 라인의 좌/우 x좌표
        y_bot: 하단 밴드의 y좌표

    Returns:
        yaw 각도(도, float) 또는 계산 불가 시 None
        - 라인이 화면에서 완벽히 수직(위아래로 평행)이면 0°
        - 위에서 아래로 가면서 오른쪽으로 기울면 양수, 왼쪽으로 기울면 음수
          (atan2 부호 규칙에 따름 - 필요시 좌표계 맞춰 조정)
    """
    if l_top is None or r_top is None or l_bot is None or r_bot is None:
        return None
    if l_top < 0 or r_top < 0 or l_bot < 0 or r_bot < 0:
        return None

    mid_top_x = (l_top + r_top) / 2.0
    mid_bot_x = (l_bot + r_bot) / 2.0

    dy = y_bot - y_top
    if dy == 0:
        return None

    return math.degrees(math.atan2(mid_bot_x - mid_top_x, dy))


def compute_horizontal_yaw(t_left, b_left, x_left, t_right, b_right, x_right):
    """
    H 모드(수평 라인)용 yaw 계산. compute_vertical_yaw과 축만 바뀐 동일 원리.

    재검증용 부호 반전 적용 중 (위 모듈 docstring "수정 이력" 참조).
    같은 물리적 회전각에 대해 compute_vertical_yaw과 같은 부호가 나오도록
    음수 반전을 넣었다. 'ver' 라인 위에서 부팅해서 초기 정렬을 마친 뒤
    'hor'로 전환했을 때 이륙/비행이 정상인지 재검증할 것.

    Args:
        t_left, b_left: 좌측 밴드에서 검출된 라인의 상/하 y좌표
        x_left: 좌측 밴드의 x좌표
        t_right, b_right: 우측 밴드에서 검출된 라인의 상/하 y좌표
        x_right: 우측 밴드의 x좌표

    Returns:
        yaw 각도(도, float) 또는 계산 불가 시 None
    """
    if t_left is None or b_left is None or t_right is None or b_right is None:
        return None
    if t_left < 0 or b_left < 0 or t_right < 0 or b_right < 0:
        return None

    mid_left_y = (t_left + b_left) / 2.0
    mid_right_y = (t_right + b_right) / 2.0

    dx = x_right - x_left
    if dx == 0:
        return None

    # 재검증용 음수 반전 (위 docstring 참조)
    return math.degrees(math.atan2(-(mid_right_y - mid_left_y), dx))


def compute_vision_yaw(mode, band_edges):
    """
    mode에 따라 알아서 V/H 계산 함수로 분기해주는 래퍼.

    Args:
        mode: 'V' 또는 'H'
        band_edges: dict, mode에 따라 필요한 키가 다름
            V 모드: {'l_top', 'r_top', 'y_top', 'l_bot', 'r_bot', 'y_bot'}
            H 모드: {'t_left', 'b_left', 'x_left', 't_right', 'b_right', 'x_right'}

    Returns:
        yaw 각도(도, float) 또는 None
    """
    if mode == 'V':
        return compute_vertical_yaw(
            band_edges['l_top'], band_edges['r_top'], band_edges['y_top'],
            band_edges['l_bot'], band_edges['r_bot'], band_edges['y_bot'],
        )
    elif mode == 'H':
        return compute_horizontal_yaw(
            band_edges['t_left'], band_edges['b_left'], band_edges['x_left'],
            band_edges['t_right'], band_edges['b_right'], band_edges['x_right'],
        )
    else:
        raise ValueError(f"Unknown mode: {mode}")


if __name__ == '__main__':
    yaw_v = compute_vertical_yaw(l_top=100, r_top=150, y_top=50,
                                  l_bot=120, r_bot=170, y_bot=500)
    print(f"V mode yaw: {yaw_v:.2f} deg")

    yaw_h = compute_horizontal_yaw(t_left=80, b_left=130, x_left=50,
                                    t_right=100, b_right=150, x_right=500)
    print(f"H mode yaw: {yaw_h:.2f} deg")

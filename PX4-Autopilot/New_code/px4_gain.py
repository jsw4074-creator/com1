############################
#     PX4 PID Parameters   #
############################

PX4_PID_PARAMS = {
    # 자세 각속도 제어기
    'MC_ROLLRATE_P': 0.15,
    'MC_ROLLRATE_I': 0.20,
    'MC_ROLLRATE_D': 0.003,

    'MC_PITCHRATE_P': 0.15,
    'MC_PITCHRATE_I': 0.20,
    'MC_PITCHRATE_D': 0.003,

    'MC_YAWRATE_P': 0.20,
    'MC_YAWRATE_I': 0.10,
    'MC_YAWRATE_D': 0.0,

    # 수평 속도 제어기
    'MPC_XY_VEL_P_ACC': 1.8,
    'MPC_XY_VEL_I_ACC': 0.4,
    'MPC_XY_VEL_D_ACC': 0.2,

    # 수직 속도 제어기
    'MPC_Z_VEL_P_ACC': 4.0,
    'MPC_Z_VEL_I_ACC': 2.0,
    'MPC_Z_VEL_D_ACC': 0.0,

    # 위치 제어기
    'MPC_XY_P': 0.95,
    'MPC_Z_P': 1.0,
}
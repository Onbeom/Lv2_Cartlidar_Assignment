"""Waffle 4WD + 팔 로봇 설정(Isaac Lab ArticulationCfg).

물리 값은 Isaac Sim 물리 시험 v12 기준 조건을 그대로 옮긴다(261022_2119/phys_tests/run_phys_test.py):
  팔 kp 400 N·m/rad, kd 36 N·m·s/rad, armature 0.045 kg·m², effort 3.8 N·m
  바퀴 kp 0, kd 1.0, armature 0.0163, effort 2.7
  왼쪽 집게 kp 1000 N/m, kd 10, 오른쪽 집게(mimic) 이득 0
  PhysX 최대 관절 속도는 1000(사실상 끔). 팔 속도 한계 4.5029 rad/s는 v12처럼 제어기(행동 항목)에서 준다.
단위는 SI(rad, m). Isaac Lab이 USD(도 단위)로 바꿔 쓴다.
"""
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

# 원본 261022_2119 USD(Physics variant physx)를 평탄화하고 base_link를 로봇 원점에 묶는 FixedJoint를 넣은 복사본
# (tools/make_rl_usd.py). articulation 루트는 base_link 그대로. 원본은 바꾸지 않는다.
# 처음 복사본(루트를 base_footprint로 옮김)은 팔 동역학이 틀려서 버림(results/usd/old_rootmoved/, 계획 문서 10장).
USD_PATH = str(Path(__file__).resolve().parents[1] / "results" / "usd" / "waffle_4wd_rl1_fixed_v2.usd")

ARM_JOINTS = ["RMX52_Base_Yaw", "RMX52_Shoulder", "RMX52_Elbow", "RMX52_Wrist_Pitch"]
ROLL_JOINT = "RMX52_Wrist_Roll_Reversed"
GRIPPER_OPEN = 0.026186130317169997      # gripper_left_joint 상한 = 열림 (m)
NO_PHYSX_VEL_LIMIT = 1000.0              # v12와 같음: PhysX 관절 속도 한계를 쓰지 않는다

WAFFLE_ARM_FIXED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        activate_contact_sensors=True,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True),  # 고정은 복사본에 있음
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "RMX52_Base_Yaw": 0.0, "RMX52_Shoulder": 1.4, "RMX52_Elbow": -1.0, "RMX52_Wrist_Pitch": -1.3,  # S0
            ROLL_JOINT: 0.0,
            "gripper_left_joint": GRIPPER_OPEN, "gripper_right_joint": -GRIPPER_OPEN,
            "Wheel_.*": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["RMX52_.*"], stiffness=400.0, damping=36.0, armature=0.045,
            effort_limit_sim=3.8, velocity_limit_sim=NO_PHYSX_VEL_LIMIT,
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["Wheel_.*"], stiffness=0.0, damping=1.0, armature=0.0163,
            effort_limit_sim=2.7, velocity_limit_sim=NO_PHYSX_VEL_LIMIT,
        ),
        "gripper_left": ImplicitActuatorCfg(
            joint_names_expr=["gripper_left_joint"], stiffness=1000.0, damping=10.0,
            effort_limit_sim=10.0, velocity_limit_sim=NO_PHYSX_VEL_LIMIT,
        ),
        "gripper_right_mimic": ImplicitActuatorCfg(
            joint_names_expr=["gripper_right_joint"], stiffness=0.0, damping=0.0,
            effort_limit_sim=10.0, velocity_limit_sim=NO_PHYSX_VEL_LIMIT,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

"""waffle_4wd 매니퓰레이터 강화 학습 과제 (팀 공유본 isaac/waffle_4wd_isaac 로봇 설정을 씀). import 하면 gym 에 등록된다.

  Waffle-SidePick-Direct  1 정렬·집기 (상자 위 퍽을 옆에서 집어 들기)
  Waffle-Store-Direct     2 옮기기·맞추기 (높이 들어 180° 돌려 뒤 스토리지 통 위에 맞춤)
  Waffle-Insert-Direct    3 넣기 (통 위 퍽을 통 안에 넣고 놓고 빼기)
  Waffle-LiePick-Direct   1' 누운 직사각형 퍽 위에서 가운데 잡아 세로로 세우기 (2026-10-30)
"""

import gymnasium as gym

from . import agents

for _id, _mod, _cfg, _agent in (
    ("Waffle-SidePick-Direct", "sidepick_env:WaffleSidePickEnv", "sidepick_env_cfg:WaffleSidePickEnvCfg", "WaffleSidePickPPORunnerCfg"),
    ("Waffle-SidePick45-Direct", "sidepick_env:WaffleSidePickEnv", "sidepick_env_cfg:WaffleSidePick45EnvCfg", "WaffleSidePick45PPORunnerCfg"),
    ("Waffle-LieEnd-Direct", "sidepick_env:WaffleLieEndEnv", "sidepick_env_cfg:WaffleLieEndEnvCfg", "WaffleLieEndPPORunnerCfg"),
    ("Waffle-SidePick45Box-Direct", "sidepick_env:WaffleSidePickEnv", "sidepick_env_cfg:WaffleSidePick45BoxEnvCfg", "WaffleSidePick45BoxPPORunnerCfg"),
    ("Waffle-Place60-Direct", "place_env:WafflePlace60Env", "place_env_cfg:WafflePlace60EnvCfg", "WafflePlace60PPORunnerCfg"),
    ("Waffle-Place45L2-Direct", "place_env:WafflePlace60Env", "place_env_cfg:WafflePlace45L2EnvCfg", "WafflePlace45L2PPORunnerCfg"),
    ("Waffle-SidePick45Chain-Direct", "sidepick_env:WaffleSidePickEnv", "sidepick_env_cfg:WaffleSidePick45ChainEnvCfg", "WaffleSidePick45ChainPPORunnerCfg"),
    ("Waffle-Unload60-Direct", "unload_env:WaffleUnloadEnv", "unload_env_cfg:WaffleUnload60EnvCfg", "WaffleUnload60PPORunnerCfg"),
    ("Waffle-Unload45-Direct", "unload_env:WaffleUnloadEnv", "unload_env_cfg:WaffleUnload45EnvCfg", "WaffleUnload45PPORunnerCfg"),
    ("Waffle-SidePickNear70-Direct", "sidepick_env:WaffleSidePickEnv", "sidepick_env_cfg:WaffleSidePickNear70EnvCfg", "WaffleSidePickNear70PPORunnerCfg"),
    ("Waffle-LieRoll45-Direct", "sidepick_env:WaffleLieEndEnv", "sidepick_env_cfg:WaffleLieRoll45EnvCfg", "WaffleLieRoll45PPORunnerCfg"),
    ("Waffle-Store-Direct", "store_env:WaffleStoreEnv", "store_env_cfg:WaffleStoreEnvCfg", "WaffleStorePPORunnerCfg"),
    ("Waffle-Insert-Direct", "insert_env:WaffleInsertEnv", "insert_env_cfg:WaffleInsertEnvCfg", "WaffleInsertPPORunnerCfg"),
    ("Waffle-LiePick-Direct", "liepick_env:WaffleLiePickEnv", "liepick_env_cfg:WaffleLiePickEnvCfg", "WaffleLiePickPPORunnerCfg"),
    ("Waffle-LieUpright-Direct", "lieupright_env:WaffleLieUprightEnv", "lieupright_env_cfg:WaffleLieUprightEnvCfg", "WaffleLieUprightPPORunnerCfg"),  # 2026-11-10 Onbeom #192
    ("Waffle-LieUprightB2-Direct", "lieupright_env:WaffleLieUprightEnv", "lieupright_env_cfg:WaffleLieUprightB2EnvCfg", "WaffleLieUprightPPORunnerCfg"),  # 2026-11-10 P1 2 회차
    ("Waffle-LieUprightB3-Direct", "lieupright_env:WaffleLieUprightEnv", "lieupright_env_cfg:WaffleLieUprightB3EnvCfg", "WaffleLieUprightPPORunnerCfg"),  # 2026-11-11 r2d 팔-상자 접촉 종료
    ("Waffle-LieUprightB4-Direct", "lieupright_env:WaffleLieUprightEnv", "lieupright_env_cfg:WaffleLieUprightB4EnvCfg", "WaffleLieUprightPPORunnerCfg"),  # 2026-11-11 r2e 옆 퍽 벌점 −60
    ("Waffle-LieUprightB6-Direct", "lieupright_env:WaffleLieUprightEnv", "lieupright_env_cfg:WaffleLieUprightB6EnvCfg", "WaffleLieUprightPPORunnerCfg"),  # 2026-11-11 r2g 노드 관측 · 속도 상한
    ("Waffle-LieUprightB5-Direct", "lieupright_env:WaffleLieUprightEnv", "lieupright_env_cfg:WaffleLieUprightB5EnvCfg", "WaffleLieUprightPPORunnerCfg"),  # 2026-11-11 r2e 2 회차 옆 퍽 여유 퍼텐셜
):
    gym.register(
        id=_id,
        entry_point=f"{__name__}.{_mod}",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.{_cfg}",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:{_agent}",
            "default_agent": "rsl_rl",
        },
    )

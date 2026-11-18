# Validation Report: 

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 14 |
| Joints | 13 |
| Assemblies | 1 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [visual_reuse]
  └─ Wheel_Revolute_Front_Left [continuous]
    wheel_front_left [BAKE] [visual_reuse]
  └─ Wheel_Revolute_Front_Right [continuous]
    wheel_front_right [BAKE] [visual_reuse]
  └─ Wheel_Revolute_Rear_Left [continuous]
    wheel_rear_left [BAKE] [visual_reuse]
  └─ Wheel_Revolute_Rear_Right [continuous]
    wheel_rear_right [BAKE] [visual_reuse]
  └─ RMX52_Base_Yaw [continuous]
    arm_link_1 [BAKE] [visual_reuse]
      └─ RMX52_Shoulder [continuous]
        arm_link_2 [BAKE] [visual_reuse]
          └─ RMX52_Elbow [continuous]
            arm_link_3 [BAKE] [visual_reuse]
              └─ RMX52_Wrist_Pitch [continuous]
                arm_link_4 [BAKE] [visual_reuse]
                  └─ RMX52_Wrist_Roll_Reversed [continuous]
                    arm_link_5 [BAKE] [visual_reuse]
                      └─ gripper_left_joint [prismatic]
                        gripper_left_link [BAKE] [visual_reuse]
                      └─ gripper_right_joint [prismatic]
                        gripper_right_link [BAKE] [visual_reuse]
  └─ lidar_front_fixed [fixed]
    lidar_front [visual_reuse]
  └─ lidar_rear_fixed [fixed]
    lidar_rear [visual_reuse]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `arm_link_1` | visual_reuse | — |
| `arm_link_2` | visual_reuse | — |
| `arm_link_3` | visual_reuse | — |
| `arm_link_4` | visual_reuse | — |
| `arm_link_5` | visual_reuse | — |
| `base_link` | visual_reuse | — |
| `gripper_left_link` | visual_reuse | — |
| `gripper_right_link` | visual_reuse | — |
| `lidar_front` | visual_reuse | — |
| `lidar_rear` | visual_reuse | — |
| `wheel_front_left` | visual_reuse | — |
| `wheel_front_right` | visual_reuse | — |
| `wheel_rear_left` | visual_reuse | — |
| `wheel_rear_right` | visual_reuse | — |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `arm_link_1` | (-87.0, 0.0, 152.4) |
| `arm_link_2` | (-87.0, -190.5, 0.0) |
| `arm_link_3` | (-111.0, -318.5, -0.0) |
| `arm_link_4` | (-235.0, -318.5, -0.0) |
| `arm_link_5` | (0.0, 318.5, 303.4) |
| `gripper_left_link` | (-361.1, -16.1, -318.5) |
| `gripper_right_link` | (-361.1, 16.1, -318.5) |
| `wheel_front_left` | (-80.0, 26.2, -139.6) |
| `wheel_front_right` | (-80.0, 26.2, 139.6) |
| `wheel_rear_left` | (80.0, 26.2, -139.6) |
| `wheel_rear_right` | (80.0, 26.2, 139.6) |

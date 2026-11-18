#  — Robot Description

## Overview

| Property | Value |
|----------|-------|
| Total mass | 4.891 kg |
| Links | 14 |
| Joints | 13 (11 movable) |
| Assemblies | 1 |
| Root link | `base_link` |

## Table of Contents

- [Kinematic Tree](#kinematic-tree)
- [Link Properties](#link-properties)
- [Joint Properties](#joint-properties)
- [Assembly Breakdown](#assembly-breakdown)
- [Quick Start (ROS 2)](#quick-start-ros-2)
- [Files](#files)

## Kinematic Tree

```
base_link
  └─ Wheel_Revolute_Front_Left [continuous]
    wheel_front_left [BAKE]
  └─ Wheel_Revolute_Front_Right [continuous]
    wheel_front_right [BAKE]
  └─ Wheel_Revolute_Rear_Left [continuous]
    wheel_rear_left [BAKE]
  └─ Wheel_Revolute_Rear_Right [continuous]
    wheel_rear_right [BAKE]
  └─ RMX52_Base_Yaw [continuous]
    arm_link_1 [BAKE]
      └─ RMX52_Shoulder [continuous]
        arm_link_2 [BAKE]
          └─ RMX52_Elbow [continuous]
            arm_link_3 [BAKE]
              └─ RMX52_Wrist_Pitch [continuous]
                arm_link_4 [BAKE]
                  └─ RMX52_Wrist_Roll_Reversed [continuous]
                    arm_link_5 [BAKE]
                      └─ gripper_left_joint [prismatic]
                        gripper_left_link [BAKE]
                      └─ gripper_right_joint [prismatic]
                        gripper_right_link [BAKE]
  └─ lidar_front_fixed [fixed]
    lidar_front
  └─ lidar_rear_fixed [fixed]
    lidar_rear
```

## Link Properties

| Link | Mass (kg) | Material | Collision | Bodies |
|------|-----------|----------|-----------|--------|
| `arm_link_1` | 0.1058 | Waffle_Equivalent_Stock_arm_link_1_Aluminum_Arm_Frame_11445_545 | visual_reuse | 51 |
| `arm_link_2` | 0.1449 | Waffle_Equivalent_Stock_arm_link_2_Aluminum_Arm_Frame_4436_486 | visual_reuse | 65 |
| `arm_link_3` | 0.1369 | Waffle_Equivalent_Stock_arm_link_3_Stock_Arm_Polymer_Estimate_1911_558 | visual_reuse | 57 |
| `arm_link_4` | 0.0964 | Waffle_Equivalent_Stock_stock_wrist_gripper_Aluminum_Arm_Frame_3305_17 | visual_reuse | 37 |
| `arm_link_5` | 0.2072 | Waffle_Equivalent_Stock_stock_wrist_gripper_Stock_Arm_Polymer_Estimate_1285_344 | visual_reuse | 262 |
| `base_link` | 3.7218 | Waffle_Equivalent_XM430_82g_1881_229 | visual_reuse | 127 |
| `gripper_left_link` | 0.0254 | Waffle_Equivalent_Stock_stock_wrist_gripper_Stock_Arm_Polymer_Estimate_1285_344 | visual_reuse | 7 |
| `gripper_right_link` | 0.0254 | Waffle_Equivalent_Stock_stock_wrist_gripper_Stock_Arm_Polymer_Estimate_1285_344 | visual_reuse | 7 |
| `lidar_front` | 0.1528 | material | visual_reuse | 24 |
| `lidar_rear` | 0.1528 | material | visual_reuse | 24 |
| `wheel_front_left` | 0.0305 | Waffle_Rubber_Tire_1100_1100_0 | visual_reuse | 2 |
| `wheel_front_right` | 0.0305 | Waffle_Rubber_Tire_1100_1100_0 | visual_reuse | 2 |
| `wheel_rear_left` | 0.0305 | Waffle_Rubber_Tire_1100_1100_0 | visual_reuse | 2 |
| `wheel_rear_right` | 0.0305 | Waffle_Rubber_Tire_1100_1100_0 | visual_reuse | 2 |

## Joint Properties

| Joint | Type | Parent → Child | Axis | Limits |
|-------|------|---------------|------|--------|
| `RMX52_Base_Yaw` | continuous | `base_link` → `arm_link_1` | (0,0,1) | — |
| `RMX52_Elbow` | continuous | `arm_link_2` → `arm_link_3` | (0,0,1) | — |
| `RMX52_Shoulder` | continuous | `arm_link_1` → `arm_link_2` | (0,0,1) | — |
| `RMX52_Wrist_Pitch` | continuous | `arm_link_3` → `arm_link_4` | (0,0,1) | — |
| `RMX52_Wrist_Roll_Reversed` | continuous | `arm_link_4` → `arm_link_5` | (0,0,1) | — |
| `Wheel_Revolute_Front_Left` | continuous | `base_link` → `wheel_front_left` | (0,0,1) | — |
| `Wheel_Revolute_Front_Right` | continuous | `base_link` → `wheel_front_right` | (0,0,1) | — |
| `Wheel_Revolute_Rear_Left` | continuous | `base_link` → `wheel_rear_left` | (0,0,1) | — |
| `Wheel_Revolute_Rear_Right` | continuous | `base_link` → `wheel_rear_right` | (0,0,1) | — |
| `gripper_left_joint` | prismatic | `arm_link_5` → `gripper_left_link` | (0,1,0) | — |
| `gripper_right_joint` | prismatic | `arm_link_5` → `gripper_right_link` | (0,1,0) | — |
| `lidar_front_fixed` | fixed | `base_link` → `lidar_front` | (0,0,1) | — |
| `lidar_rear_fixed` | fixed | `base_link` → `lidar_rear` | (0,0,1) | — |

## Assembly Breakdown

### main

- **Links**: arm_link_1, arm_link_2, arm_link_3, arm_link_4, arm_link_5, base_link, gripper_left_link, gripper_right_link, lidar_front, lidar_rear, wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right
- **Total mass**: 4.891 kg

## Quick Start (ROS 2)

```bash
# 1. Copy package to your ROS 2 workspace
cp -r waffle_4wd_description_261021 ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select waffle_4wd_description_261021
source install/setup.bash

# 3. Visualize in RViz2
ros2 launch waffle_4wd_description_261021 display.launch.py

# 4. Validate URDF structure
check_urdf install/waffle_4wd_description_261021/share/waffle_4wd_description_261021/urdf/waffle_4wd.urdf

# 5. Print kinematic tree
urdf_to_graphviz install/waffle_4wd_description_261021/share/waffle_4wd_description_261021/urdf/waffle_4wd.urdf
```

**Joint control**: The launch file includes `joint_state_publisher_gui` —
use the sliders to move revolute/prismatic joints in RViz2.

**Topic inspection**:
```bash
# See published joint states
ros2 topic echo /joint_states

# See robot description parameter
ros2 param get /robot_state_publisher robot_description
```

## Files

| Path | Description |
|------|-------------|
| `urdf/waffle_4wd.urdf.xacro` | Top-level xacro (entry point) |
| `urdf/waffle_4wd.urdf` | Flat URDF (for validation) |
| `urdf/assemblies/` | Per-assembly xacro macros |
| `meshes/` | Visual (OBJ) and collision (STL) meshes |
| `launch/display.launch.py` | Launch robot_state_publisher, RViz, and generated controllers |
| `config/joint_state.yaml` | Joint state publisher config |
| `robot_data.yaml` | Supplementary data (beyond URDF) |
| `docs/transforms.md` | Transformation matrices (KaTeX) |

## Customizing

Assemblies tagged `!dummy_` are designed to be swapped out. To replace one:

1. Create your replacement as a xacro macro with the same interface
2. Place it in `urdf/assemblies/`
3. Update the `<xacro:include>` in `urdf/waffle_4wd.urdf.xacro`
4. Update meshes in `meshes/<your_assembly>/`

The xacro prefix system (`${prefix}`) ensures link names stay unique
when multiple instances of the same assembly are used.

---
*Generated by Fusion URDF/XACRO Exporter v3.1.1*
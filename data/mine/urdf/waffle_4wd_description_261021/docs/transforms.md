> HISTORICAL generic-export baseline. The current package has23 links/22 joints and root base_footprint. See simulation_metadata.json and actual URDF. Do not regenerate/reframe without preserving bounded limits, efforts, collision, mimic and sensor frames.

# Transformation Matrices - 

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | wheel_front_left |
| $L_{2}$ | wheel_front_right |
| $L_{3}$ | wheel_rear_left |
| $L_{4}$ | wheel_rear_right |
| $L_{5}$ | arm_link_1 |
| $L_{6}$ | lidar_front |
| $L_{7}$ | lidar_rear |
| $L_{8}$ | arm_link_2 |
| $L_{9}$ | arm_link_3 |
| $L_{10}$ | arm_link_4 |
| $L_{11}$ | arm_link_5 |
| $L_{12}$ | gripper_left_link |
| $L_{13}$ | gripper_right_link |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Wheel_Revolute_Front_Left | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Wheel_Revolute_Front_Right | continuous (rad) | $L_{0}$ | $L_{2}$ |
| $q_{3}$ | Wheel_Revolute_Rear_Left | continuous (rad) | $L_{0}$ | $L_{3}$ |
| $q_{4}$ | Wheel_Revolute_Rear_Right | continuous (rad) | $L_{0}$ | $L_{4}$ |
| $q_{5}$ | RMX52_Base_Yaw | continuous (rad) | $L_{0}$ | $L_{5}$ |
| $q_{6}$ | RMX52_Shoulder | continuous (rad) | $L_{5}$ | $L_{8}$ |
| $q_{7}$ | RMX52_Elbow | continuous (rad) | $L_{8}$ | $L_{9}$ |
| $q_{8}$ | RMX52_Wrist_Pitch | continuous (rad) | $L_{9}$ | $L_{10}$ |
| $q_{9}$ | RMX52_Wrist_Roll_Reversed | continuous (rad) | $L_{10}$ | $L_{11}$ |
| $q_{10}$ | gripper_left_joint | prismatic (m) | $L_{11}$ | $L_{12}$ |
| $q_{11}$ | gripper_right_joint | prismatic (m) | $L_{11}$ | $L_{13}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] Wheel_Revolute_Front_Left (q1)
  |   L1: wheel_front_left
  |-- [continuous] Wheel_Revolute_Front_Right (q2)
  |   L2: wheel_front_right
  |-- [continuous] Wheel_Revolute_Rear_Left (q3)
  |   L3: wheel_rear_left
  |-- [continuous] Wheel_Revolute_Rear_Right (q4)
  |   L4: wheel_rear_right
  |-- [continuous] RMX52_Base_Yaw (q5)
  |   L5: arm_link_1
  |     +-- [continuous] RMX52_Shoulder (q6)
  |         L8: arm_link_2
  |           +-- [continuous] RMX52_Elbow (q7)
  |               L9: arm_link_3
  |                 +-- [continuous] RMX52_Wrist_Pitch (q8)
  |                     L10: arm_link_4
  |                       +-- [continuous] RMX52_Wrist_Roll_Reversed (q9)
  |                           L11: arm_link_5
  |                             |-- [prismatic] gripper_left_joint (q10)
  |                             |   L12: gripper_left_link
  |                             +-- [prismatic] gripper_right_joint (q11)
  |                                 L13: gripper_right_link
  |-- [fixed] lidar_front_fixed
  |   L6: lidar_front
  +-- [fixed] lidar_rear_fixed
      L7: lidar_rear
```

## Transforms

## Wheel_Revolute_Front_Left

$L_{0}$ **base_link** -> $L_{1}$ **wheel_front_left** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0.08, 0.1396, 0.02625) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.08 \\
0 & 0 & 1 & 0.1396 \\
0 & -1 & 0 & 0.02625 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Wheel_Revolute_Front_Right

$L_{0}$ **base_link** -> $L_{2}$ **wheel_front_right** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.08, -0.1396, 0.02625) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.08 \\
0 & 0 & 1 & -0.1396 \\
0 & -1 & 0 & 0.02625 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Wheel_Revolute_Rear_Left

$L_{0}$ **base_link** -> $L_{3}$ **wheel_rear_left** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (-0.08, 0.1396, 0.02625) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.08 \\
0 & 0 & 1 & 0.1396 \\
0 & -1 & 0 & 0.02625 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Wheel_Revolute_Rear_Right

$L_{0}$ **base_link** -> $L_{4}$ **wheel_rear_right** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (-0.08, -0.1396, 0.02625) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.08 \\
0 & 0 & 1 & -0.1396 \\
0 & -1 & 0 & 0.02625 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RMX52_Base_Yaw

$L_{0}$ **base_link** -> $L_{5}$ **arm_link_1** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0.087, 0, 0.15235) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.087 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & 0.15235 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## lidar_front_fixed

$L_{0}$ **base_link** -> $L_{6}$ **lidar_front** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{6} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## lidar_rear_fixed

$L_{0}$ **base_link** -> $L_{7}$ **lidar_rear** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{7} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RMX52_Shoulder

$L_{5}$ **arm_link_1** -> $L_{8}$ **arm_link_2** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (0, 0, -0.03815) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{8}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & -1 & 0 & -0.03815 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RMX52_Elbow

$L_{8}$ **arm_link_2** -> $L_{9}$ **arm_link_3** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0.024, 0.128, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{8}_{9}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0.024 \\
s_{7} & c_{7} & 0 & 0.128 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RMX52_Wrist_Pitch

$L_{9}$ **arm_link_3** -> $L_{10}$ **arm_link_4** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0.124, 0, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{9}_{10}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0.124 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RMX52_Wrist_Roll_Reversed

$L_{10}$ **arm_link_4** -> $L_{11}$ **arm_link_5** (continuous)
  Variable: $q_{9}$

- **origin xyz**: (0.0684, 0, 0) m
- **origin rpy**: (-3.141593, 1.570796, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{10}_{11}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.0684 \\
0 & -1 & 0 & 0 \\
-1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & -s_{9} & 0 & 0 \\
s_{9} & c_{9} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## gripper_left_joint

$L_{11}$ **arm_link_5** -> $L_{12}$ **gripper_left_link** (prismatic)
  Variable: $q_{10}$

- **origin xyz**: (0.016114, 0, -0.0577) m
- **origin rpy**: (1.570796, 1.570796, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$$
T^{11}_{12}(q_{10}) = \begin{bmatrix}
0 & 1 & 0 & 0.016114 \\
0 & 0 & -1 & q_{10} \\
-1 & 0 & 0 & -0.0577 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## gripper_right_joint

$L_{11}$ **arm_link_5** -> $L_{13}$ **gripper_right_link** (prismatic)
  Variable: $q_{11}$

- **origin xyz**: (-0.016114, 0, -0.0577) m
- **origin rpy**: (1.570796, 1.570796, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$$
T^{11}_{13}(q_{11}) = \begin{bmatrix}
0 & 1 & 0 & -0.016114 \\
0 & 0 & -1 & q_{11} \\
-1 & 0 & 0 & -0.0577 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{8} = T^{0}_{5}(q_{5}) \cdot T^{5}_{8}(q_{6})\quad (L_0 \to L_{8}: \text{arm_link_2})$$

$$T^{0}_{9} = T^{0}_{5}(q_{5}) \cdot T^{5}_{8}(q_{6}) \cdot T^{8}_{9}(q_{7})\quad (L_0 \to L_{9}: \text{arm_link_3})$$

$$T^{0}_{10} = T^{0}_{5}(q_{5}) \cdot T^{5}_{8}(q_{6}) \cdot T^{8}_{9}(q_{7}) \cdot T^{9}_{10}(q_{8})\quad (L_0 \to L_{10}: \text{arm_link_4})$$

$$T^{0}_{11} = T^{0}_{5}(q_{5}) \cdot T^{5}_{8}(q_{6}) \cdot T^{8}_{9}(q_{7}) \cdot T^{9}_{10}(q_{8}) \cdot T^{10}_{11}(q_{9})\quad (L_0 \to L_{11}: \text{arm_link_5})$$

$$T^{0}_{12} = T^{0}_{5}(q_{5}) \cdot T^{5}_{8}(q_{6}) \cdot T^{8}_{9}(q_{7}) \cdot T^{9}_{10}(q_{8}) \cdot T^{10}_{11}(q_{9}) \cdot T^{11}_{12}(q_{10})\quad (L_0 \to L_{12}: \text{gripper_left_link})$$

$$T^{0}_{13} = T^{0}_{5}(q_{5}) \cdot T^{5}_{8}(q_{6}) \cdot T^{8}_{9}(q_{7}) \cdot T^{9}_{10}(q_{8}) \cdot T^{10}_{11}(q_{9}) \cdot T^{11}_{13}(q_{11})\quad (L_0 \to L_{13}: \text{gripper_right_link})$$


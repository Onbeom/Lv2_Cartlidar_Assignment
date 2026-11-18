# Waffle 4WD description — 261022 revision (based on 261021 simulation inputs)

Revision 261022 (2026-10-22): package renamed to `waffle_4wd_description_261022`; four gripper fastener sets (8 pieces, 4724 triangles) removed from the arm_link_5 visual mesh only. arm_link_5 visual front end (x max, base_footprint frame) changed from 366.6 mm to 361.1 mm. URDF joints, collision meshes, mass and inertia are unchanged from 261021. See REVISION_261022.md.

Updated in the existing package. 23 links / 22 joints, including14 physical links and9 massless fixed logical frames. Root: base_footprint. Empty estimated mass4891.1940677g;12 solid-PLA pucks add803.52g, total5694.7140677g. Payload is not baked into robot inertials.

Engineering data uses mm/g/g·mm². URDF uses mandatory m/kg/kg·m²/rad. Existing CAD engineering coordinates are relative to base_link; base_link is6.7499816754mm above footprint ground.

Arm reference limits are bounded revolute, not measured safe hard stops. Efforts3.8N·m(arm) and2.7N·m(wheels) are manufacturer11.1V STALL peak values, not continuous safe torque. Each OpenCR has its own TurtleBot11.1V nominal battery; Jetson has separate12V power. Equivalent finger effort10N each, velocities and damping are provisional assumptions. Right finger mimics left with multiplier−1, aperture16.8..72mm. Actual crank calibration/force remains unknown.

Collision: tire cylinders, simplified structural/arm boxes, convex meshes for cavity-sensitive storage, rails, brackets and fingers. The1mm decomposition setting is a target, not certified achieved error. Tiny internal details/fasteners omitted. Validate PhysX contacts, filtering and self-collision in the actual runtime.

Added UC684 and front/rear top camera optical frames, front/rear LDS scan frames. These use CAD datums/assumptions, not calibrated optical centers. See simulation_metadata.json for full basis. No sensor plugins, ros2_control hardware interfaces, actuator drives, contact materials or runtime-specific Isaac articulation settings are asserted configured.

Static checks: simulation_audit.json verifies tree, SI conversions, retained mass/CoM/tensors/visual bounds (audit performed on 261021; in 261022 the arm_link_5 visual bounds differ as noted above), axes, bounded limits, closed convex STL files and footprint ground offset. ROS, RViz and Isaac Sim/Lab have NOT been executed here. Historical402-frame G-channel trajectory has reference-limit violations and requires replanning or measured limit validation before bounded-joint playback/hardware use.

ROS2 display after dependencies and build: `ros2 launch waffle_4wd_description_261022 display.launch.py`. Validate xacro/check_urdf and mimic handling in your target installation first. The native F3D export copy remains14 physical links/13 native joints; added logical frames are URDF postprocessing. Re-exporting generically requires reapplying this configuration.

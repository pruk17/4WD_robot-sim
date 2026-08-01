# robot_4wd — 4-wheel swerve-drive robot (Gazebo Harmonic + ROS 2 Jazzy)

Prototype robot for the LiDAR-simulation assignment. Simple box/cylinder links
(no meshes yet), swerve drive, 2D LiDAR, runs in an empty world.

## Robot spec
- **base_link**: box 0.45 × 0.45 × 0.15 m
- **4 swerve modules** at the corners: box 0.12 × 0.12 × 0.15 m each,
  every module = a *steer* joint (revolute, about Z) + a *drive* wheel
  (continuous). 8 actuated joints total.
- **wheels**: 5″ diameter (radius 0.0635 m)
- **LiDAR**: 2D `gpu_lidar`, 360 samples, 0.2–12 m, on top of the chassis

## File map
```
description/robot.urdf.xacro   master (includes the rest)
description/inertial_macros.xacro   inertia helper macros
description/robot_core.xacro   base_footprint + chassis + colors
description/swerve_module.xacro macro for 1 module, instantiated ×4 (fl/fr/bl/br)
description/lidar.xacro         lidar link + gpu_lidar sensor
description/gazebo_control.xacro gz joint controllers + joint-state publisher
worlds/empty.sdf                empty world (physics + sensors + ground + sun)
config/gz_bridge.yaml           ROS <-> GZ topic bridge
config/view.rviz                RViz layout (robot + TF + laser)
launch/launch_sim.launch.py     brings up everything
robot_4wd/swerve_controller.py  /cmd_vel -> per-wheel steer + drive (swerve IK)
```

## Build
```bash
cd ~/4WD_robot_ws
colcon build --packages-select robot_4wd      # do NOT use --symlink-install (breaks data_files)
source install/setup.bash
```

## Run
```bash
# Gazebo GUI (shows the LiDAR beams). Add rviz:=true to also open RViz.
ros2 launch robot_4wd launch_sim.launch.py
ros2 launch robot_4wd launch_sim.launch.py rviz:=true
ros2 launch robot_4wd launch_sim.launch.py gui:=false   # headless
```

## Drive it (new terminal)
```bash
source ~/4WD_robot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Because this is **swerve (holonomic)** the robot can strafe sideways:
- `i/,` forward / back  → linear.x
- `j/l` rotate          → angular.z
- press `Shift` variants `J/L` (holonomic mode) → strafe left/right → linear.y

## Topics
| topic | dir | meaning |
|---|---|---|
| `/cmd_vel` | in  | body twist (x=fwd, y=strafe, z=yaw) |
| `/scan`    | out | LiDAR (2D). Verified: detects a box at 3 m as min_range 2.5 m |
| `/joint_states` | out | wheel + steer angles |
| `/{fl,fr,bl,br}_steer_cmd` | out | per-module steering angle |
| `/{fl,fr,bl,br}_drive_cmd` | out | per-module wheel speed |

## Notes / tuning
- No off-the-shelf swerve plugin exists, so the IK is done in
  `swerve_controller.py`; each joint is driven by a gz `JointPositionController`
  (steer) / `JointController` (drive).
- If the robot drives the *wrong way*, flip the sign of `omega` in
  `swerve_controller.py`, or the wheel `rpy` in `swerve_module.xacro`.
- Steering feels sluggish/oscillates → tune `p_gain/d_gain` in
  `gazebo_control.xacro`.
- The `gz_frame_id` SDF warning is harmless (fixed-joint lumping).
- **NVIDIA Optimus laptops**: the LiDAR (gpu_lidar) needs an offscreen EGL
  context on the discrete GPU. The launch sets `__NV_PRIME_RENDER_OFFLOAD` /
  `__GLX_VENDOR_LIBRARY_NAME` / `__EGL_VENDOR_LIBRARY_FILENAMES` automatically
  (`use_nvidia:=true`, the default). Without it you'd see
  `libEGL warning: ... failed to create dri2 screen` and an empty `/scan`.
  On a non-NVIDIA machine run with `use_nvidia:=false`.
```

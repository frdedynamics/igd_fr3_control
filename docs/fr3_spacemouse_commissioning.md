# FR3 SpaceMouse Commissioning

This document records the working configuration used to commission direct
SpaceMouse teleoperation of the Franka FR3 with MoveIt Servo and the Franka
Hand.

## Tested environment

- Ubuntu 24.04
- ROS 2 Jazzy
- PREEMPT_RT kernel
- FR3 robot: `192.170.10.101`
- Host FR3 interface: `192.170.10.95/24`
- Direct built-in Ethernet connection to the robot
- Workspace: `~/franka_ros2_ws`

The ROS user has realtime privileges and unlimited locked memory.

## Required Franka ROS 2 compatibility changes

The tested system uses `franka_ros2` v3.0.0 (`ad6631e`) with two local
backports:

1. Controller manager configuration:

   ```yaml
   thread_priority: 98
   enable_overrun: false
   ```

2. Correct gripper joint-state source:

    ```yaml
    franka_gripper/joint_states
    ```

These changes were required for the tested Jazzy setup. The controller-manager
settings are present in later `franka_ros2` releases, and the gripper topic is
corrected on the newer Jazzy branch.

The previous ROS control communication dropout was not reproduced during more
than one hour of operation after commissioning. This should not be interpreted
as proof that a single change was the root cause; direct Ethernet, realtime
configuration, memory locking, and the controller-manager backport were changed
during the same commissioning process.


## Build


Because the local environment contains a user-site setuptools version that
conflicts with the ROS Python build environment, build this package with:

```bash
    cd ~/franka_ros2_ws
    PYTHONNOUSERSITE=1 colcon build --packages-select igd_fr3_control
```


Then source the workspace:

```bash
    source /opt/ros/jazzy/setup.bash
    source ~/franka_ros2_ws/install/setup.bash
```

## FR3 bringup

Start the robot and MoveIt:

```bash
    ros2 launch franka_fr3_moveit_config moveit.launch.py \
      robot_ip:=192.170.10.101 \
      robot_type:=fr3 \
      use_fake_hardware:=false
```

The merged `/joint_states` topic must contain the seven FR3 joints and both
gripper finger joints.

## MoveIt Servo

Start Servo separately:

```bash
    ros2 launch igd_fr3_control fr3_spacemouse_servocontrol.launch.py
```

The SpaceMouse configuration uses:

- command type: unitless Cartesian twist
- input topic: `/servo_node/delta_twist_cmds`
- command frame: `fr3_hand_tcp`
- output rate: 50 Hz
- joint state input: `/joint_states`
- linear scale: `0.25 m/s`
- rotational scale: `0.50 rad/s`

For the installed MoveIt Servo version the scale parameters use the nested
schema:

```yaml
scale:
  linear: 0.25
  rotational: 0.50
```

## SpaceMouse

Start the publisher with:

```bash
    ros2 run igd_fr3_control spacemouse_twiststamped_publisher \
      --ros-args \
      -p topic:=/servo_node/delta_twist_cmds
```

The SpaceMouse translation and rotation mappings were calibrated independently
against physical FR3 motion. All three translation axes and all three rotation
axes were verified manually.

Button 1 toggles the Franka Hand:

- open -> close
- closed -> open

The toggle uses the Franka `Move` action and determines the current state from
`/franka_gripper/joint_states`. Button handling is edge-triggered and action
server checks are non-blocking, so a button press cannot stall Cartesian
command publication.

Button 0 is currently unused.

## Validation

The following were physically verified:

FR3 MoveIt/RViz motion
SpaceMouse XYZ translation
SpaceMouse XYZ rotation
fine Cartesian control using scales `0.25 / 0.50`
Franka Hand open/close toggle
continued SpaceMouse operation after button presses
stable ROS control operation for more than one hour without reproducing the
earlier communication dropout

An independent libfranka communication test also completed successfully with
an average command success rate of `1.00`.

## Safety

MoveIt Servo collision checking, singularity handling, and joint limits are
software safeguards only. They do not replace independent physical safety
mechanisms, workspace supervision, appropriate speed/force limits, or an
accessible stop/E-stop.


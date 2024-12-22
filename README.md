# whi_moveit_cpp_bridge
Bridge the MoveIt planning and execution with the MoveItCpp interface offering both message and service interfaces

![moveitcpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/29b0b522-7429-4401-9c42-54f7970dd4b3)

## Dependency
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Advertised topic
**tcp_pose**(whi_interfaces::WhiTcpPose)

For quick validation, input the following command with the configured pose group:
```
rostopic pub -1 /whi_moveit_cpp_bridge/tcp_pose whi_interfaces/WhiTcpPose "{pose_group: 'home'}"
```

> NOTE: please replace the pose_group with your configured pose group

Or with an absolute pose in the world:
```
rostopic pub -1 /whi_moveit_cpp_bridge/tcp_pose whi_interfaces/WhiTcpPose "{tcp_pose: {header: {frame_id: ''}, pose:{position: {x: 0.2, y: 0.349, z: 0.9128}, orientation: {x: -0.707107, y: 0.0, z: 0.0, w: 0.707107}}}}"
```

![cpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/eea78e20-2895-4d4e-8436-d42a17aef736)


**joint_pose**(whi_interfaces::WhiJointPose)

An example of the absolute positions for the joint group:
```
rostopic pub -1 /whi_moveit_cpp_bridge/joint_pose whi_interfaces/WhiJointPose "{joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}, velocity_scale: 0.05, is_relative: false}"
```
Or with the relative positions:
```
rostopic pub -1 /whi_moveit_cpp_bridge/joint_pose whi_interfaces/WhiJointPose "{joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}, velocity_scale: 0.05, is_relative: true}"
```

## Subscribed topic
**estop**(std_msgs::Bool)

To stop the current executing trajectory

## Advertised service
**tcp_pose**(whi_interfaces::WhiSrvTcpPose)

Like Published topic use the rosservice command line to make a quick validation:
```
rosservice call /whi_moveit_cpp_bridge/tcp_pose "{pose: {pose_group: 'up', velocity_scale: 0.05}}"

```

and the following for pose:
```
rosservice call /whi_moveit_cpp_bridge/tcp_pose "{pose: {tcp_pose: {header: {frame_id: 'camera'}, pose:{position: {x: 0.0, y: 0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity_scale: 0.05}}"
```

> NOTE: please replace the pose_group and the position/orientation with your configured ones respectively

> TIP: use the Matlab online to calculate the quaternion: https://www.mathworks.com/help/nav/ref/eul2quat.html


**joint_pose**(whi_interfaces::WhiSrvJointPose)

An example of the absolute positions for the joint group:
```
rosservice call /whi_moveit_cpp_bridge/joint_pose "{pose: {joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}, velocity_scale: 0.05, is_relative: false}}"
```
Or with the relative positions:
```
rosservice call /whi_moveit_cpp_bridge/joint_pose "{pose: {joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}, velocity_scale: 0.05, is_relative: true}}"
```

**joint_names**(whi_interfaces::WhiJointNames)

Use this service to check the sequence of each joint in the joint group, if it is uncertain:
```
rosservice call /whi_moveit_cpp_bridge/joint_names
```
![image](https://github.com/user-attachments/assets/6c5ff956-2900-4b1c-b630-b2bfb019c666)

**tcp_difference**(whi_interfaces::WhiSrvTcpDifference)

Get the offset of TCP between the current state and a specified reference state. The reference state has three types: 1 pose group, 2 joint position, 3 TCP pose

An example of the difference to the pose group:
```
rosservice call /whi_moveit_cpp_bridge/tcp_difference "{pose_group: 'ready_inspection'}"
```

An example of the difference to the joint position:
```
rosservice call /whi_moveit_cpp_bridge/tcp_difference "{joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}}"
```

An example of the difference to the TCP pose:
```
rosservice call /whi_moveit_cpp_bridge/tcp_difference "{tcp_pose: {pose:{position: {x: 0.0, y: 0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

**tcp_current**(whi_interfaces::WhiSrvCurrentTcpPose)

Use this service to check the current TCP pose:
```
rosservice call /whi_moveit_cpp_bridge/tcp_current
```
![image](https://github.com/user-attachments/assets/a09a0009-149d-488f-bcf6-49b747538733)

## Usage
For a quick validation, set the argument "controller" to "fake", for controlling a real arm, please refer to the arm's hardware interface for its controller name:

### Fake controller
```
# UR10e
roslaunch whi_moveit_cpp_bridge whi_moveit_cpp_bridge.launch arm:=ur arm_model:=10e controller:=fake
# Chin CRB7
roslaunch whi_moveit_cpp_bridge whi_moveit_cpp_bridge.launch arm:=chin arm_model:=crb7 controller:=fake
```

### Real hardware
```
# UR10e
roslaunch whi_moveit_cpp_bridge whi_moveit_cpp_bridge.launch arm:=ur arm_model:=10e controller:=ur10e
# Chin CRB7
roslaunch whi_moveit_cpp_bridge whi_moveit_cpp_bridge.launch arm:=chin arm_model:=crb7 controller:=chin_crb7
```

## Params
```
whi_moveit_cpp_bridge:
  planning_group: manipulator
  max_ik_try_count: 30
  tcp_pose_service: tcp_pose
  tcp_pose_topic: tcp_pose
  motion_state_topic: arm_motion_state
```

The param "motion_state_topic" creates the subscriber to receive the message whether the arm enters the protective stop state.

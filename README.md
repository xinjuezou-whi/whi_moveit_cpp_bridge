# whi_moveit_cpp_bridge
Bridge the MoveIt planning and execution with the MoveItCpp interface offering both message and service interfaces

![moveitcpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/29b0b522-7429-4401-9c42-54f7970dd4b3)

## Dependency
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Advertised topic
**tcp_pose**(whi_interfaces::msg::WhiTcpPose)

For quick validation, input the following command with the configured pose group:
```
ros2 topic pub -1 /tcp_pose whi_interfaces/msg/WhiTcpPose "{pose_group: 'up', velocity_scale: 0.05}"
```

> NOTE: please replace the pose_group with your configured pose group

Or with an absolute pose of a given link(frame) in the world, bellowing example is the pose of end effector `tool0`:
```
ros2 topic pub -1 /tcp_pose whi_interfaces/msg/WhiTcpPose "{tcp_pose: {header: {frame_id: 'tool0'}, pose:{position: {x: 0.2, y: 0.349, z: 0.9128}, orientation: {x: -0.707107, y: 0.0, z: 0.0, w: 0.707107}}}, velocity_scale: 0.05}"
```

![cpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/eea78e20-2895-4d4e-8436-d42a17aef736)


**joint_pose**(whi_interfaces::msg::WhiJointPose)

An example of the absolute positions for the joint group:
```
ros2 topic pub -1 /joint_pose whi_interfaces/msg/WhiJointPose "{joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}, velocity_scale: 0.05, is_relative: false}"
```
Or with the relative positions:
```
ros2 topic pub -1 /joint_pose whi_interfaces/msg/WhiJointPose "{joint_pose: {position: [0, 0, 0, 0, 0, 1.5707]}, velocity_scale: 0.05, is_relative: true}"
```

## Subscribed topic
**estop**(std_msgs::msg::Bool)

To stop the current executing trajectory

## Advertised service
**tcp_pose**(whi_interfaces::msg::WhiSrvTcpPose)

Like Published topic use the rosservice command line to make a quick validation:
```
ros2 service call /tcp_pose whi_interfaces/srv/WhiSrvTcpPose "{pose: {pose_group: 'up', velocity_scale: 0.05}}"
```

and the following for pose:
```
ros2 service call /tcp_pose whi_interfaces/srv/WhiSrvTcpPose "{pose: {tcp_pose: {header: {frame_id: 'tool0'}, pose:{position: {x: 0.0, y: 0.05, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity_scale: 0.05}}"
```
Or with Cartesian:
```
ros2 service call /tcp_pose whi_interfaces/srv/WhiSrvTcpPose "{pose: {tcp_pose: {header: {frame_id: 'tool0'}, pose:{position: {x: 0.0, y: 0.05, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity_scale: 0.05, is_cartesian: true}}"
```

> NOTE: please replace the pose_group and the position/orientation with your configured ones respectively

> TIP: use the Matlab online to calculate the quaternion: https://www.mathworks.com/help/nav/ref/eul2quat.html


**joint_pose**(whi_interfaces::srv::WhiSrvJointPose)

An example of the absolute positions for the joint group:
```
ros2 service call /joint_pose whi_interfaces/srv/WhiSrvJointPose "{pose: {joint_pose: {position: [0, 0, 0, 0, 0, 0.7854]}, velocity_scale: 0.05, is_relative: false}}"
```
Or with the relative positions:
```
ros2 service call /joint_pose whi_interfaces/srv/WhiSrvJointPose "{pose: {joint_pose: {position: [0, 0, 0, 0, 0, 0.7854]}, velocity_scale: 0.05, is_relative: true}}"
```

**joint_names**(whi_interfaces::srv::WhiJointNames)

Use this service to check the sequence of each joint in the joint group, if it is uncertain:
```
ros2 service call /joint_names whi_interfaces/srv/WhiSrvJointNames
```
![image](https://github.com/user-attachments/assets/6c5ff956-2900-4b1c-b630-b2bfb019c666)

**tcp_difference**(whi_interfaces::srv::WhiSrvTcpDifference)

Get the offset of TCP between the current state and a specified reference state. The reference state has three types: 1 pose group, 2 joint position, 3 TCP pose

An example of the difference to the pose group:
```
ros2 service call /tcp_difference whi_interfaces/srv/WhiSrvTcpDifference "{pose_group: {header: {frame_id: 'tool0'}, pose_group: 'ready_inspection'}"
```

An example of the difference to the joint position:
```
ros2 service call /tcp_difference whi_interfaces/srv/WhiSrvTcpDifference "{joint_pose: {header: {frame_id: 'tool0'}, {position: [0, 0, 0, 0, 0, 1.5707]}}"
```

An example of the difference to the TCP pose:
```
ros2 service call /tcp_difference whi_interfaces/srv/WhiSrvTcpDifference "{tcp_pose: {header: {frame_id: 'tool0'}, {pose:{position: {x: 0.0, y: 0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

**tcp_current**(whi_interfaces::srv::WhiSrvCurrentTcpPose)

Use this service to check the current TCP pose:
```
ros2 service call /tcp_current whi_interfaces/srv/WhiSrvCurrentTcpPose "{header: {frame_id: 'tool0'}}"
```
![image](https://github.com/user-attachments/assets/a09a0009-149d-488f-bcf6-49b747538733)


**joint_current**(whi_interfaces::srv::WhiSrvCurrentJointPose)

Use this service to check the current TCP pose:
```
ros2 service call /joint_current whi_interfaces/srv/WhiSrvCurrentJointPose "{}"
```
<img width="454" height="142" alt="image" src="https://github.com/user-attachments/assets/5bec6431-1002-4808-9d49-b5b3494912a1" />


**abort_execution**(std_srvs::srv::Trigger)

To abort the current execution:
```
ros2 service call /abort_execution std_srvs/srv/Trigger "{}"
```

## Usage
For a quick validation, set the argument "controller" to "fake", for controlling a real arm, please refer to the arm's hardware interface for its controller name:

### Fake controller
```
# UR5e
# launch ur_robot_driver firstly
ros2 launch ur_robot_driver ur5e.launch.py robot_ip:=192.168.56.100 use_mock_hardware:=true
# then launch moveit_cpp_bridge
ros2 launch whi_moveit_cpp_bridge launch.py arm:=ur arm_model:=5e
```

### Real hardware
```
# UR5e
# launch ur_robot_driver firstly
ros2 launch ur_robot_driver ur5e.launch.py robot_ip:=192.168.56.100
# then launch moveit_cpp_bridge
ros2 launch whi_moveit_cpp_bridge launch.py arm:=ur arm_model:=5e
```

## Params
```
whi_moveit_cpp_bridge:
  ros__parameters:
    arm_ready_service: arm_ready
    estop_topic: estop
    motion_state_topic: motion_state
    wait_duration: 1.0 # second
    max_try_count: 30
    cartesian_fraction: 0.95
    cartesian_traj_max_step: 0.1
    cartesian_precision: [0.005, 0.01]

```

The param "motion_state_topic" creates the subscriber to receive the message whether the arm enters the protective stop state.

# whi_moveit_cpp_bridge
Bridge the MoveIt planning and execution with the MoveItCpp interface offering both message and service interfaces

![moveitcpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/29b0b522-7429-4401-9c42-54f7970dd4b3)

## Dependency
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Published topic
**tcp_pose**(whi_interfaces::WhiTcpPose)

For quick validation, input the following command with configured pose group:
```
rostopic pub -1 /whi_moveit_cpp_bridge/tcp_pose whi_interfaces/WhiTcpPose "{pose_group: 'home'}"
```

> NOTE: please replace the pose_group with your configured pose group

Or with an absolute pose in the world:
```
rostopic pub -1 /whi_moveit_cpp_bridge/tcp_pose whi_interfaces/WhiTcpPose "{tcp_pose: {header: {frame_id: ''}, pose:{position: {x: 0.2, y: 0.349, z: 0.9128}, orientation: {x: -0.707107, y: 0.0, z: 0.0, w: 0.707107}}}}"
```

![cpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/eea78e20-2895-4d4e-8436-d42a17aef736)

## Advertised service
**tcp_pose**(whi_interfaces::WhiSrvTcpPose)

Like Published topic use the rosservice command line to make a quick validation:
```
rosservice call /whi_moveit_cpp_bridge/tcp_pose "{pose_group: 'ready'}"
```

and the following for pose:
```
rosservice call /whi_moveit_cpp_bridge/tcp_pose "{tcp_pose: {header: {frame_id: 'camera'}, pose:{position: {x: 0.0, y: 0.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

> NOTE: please replace the pose_group and the position/orientation with your configured ones respectively

# whi_moveit_cpp_bridge
Bridge the MoveIt planning and execution with the MoveItCpp interface offering both message and service interfaces

![moveitcpp_bridge](https://github.com/xinjuezou-whi/whi_moveit_cpp_bridge/assets/72239958/29b0b522-7429-4401-9c42-54f7970dd4b3)

## Dependency
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Published topic
**tcp_pose**(whi_interfaces::WhiTcpPose)

for validation input the following command:
```
rostopic pub -1 /whi_moveit_cpp_bridge/tcp_pose whi_interfaces/WhiTcpPose "{pose_group: 'home'}"
```
> NOTE: please replace the pose_group with your configured pose group

## Advertised service
**tcp_pose**(whi_interfaces::WhiSrvTcpPose)

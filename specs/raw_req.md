
## 目标需求
实现主从臂跟随, 移动主臂，从臂能够快速响应跟随主臂移动
1. 实现移动真实主臂，Gazebo仿真环境中从臂响应并移动
2. 实现移动真实主臂，真实环境中从臂响应并移动
3. 所有代码及依赖都拷贝到此项目中, 例如URDF/Mesh文件等


## 当前状态
现在已经实现了两个机械臂的数字孪生，rviz2 能够正确反馈真实机械臂状态


1. 主臂数字孪生实现: /Users/jialiang/Project/Star-Arm-102/ROS2_HUMBLE/src/stararm102_description/launch/digital_twin.launch.py

2. 从臂数字孪生实现: /Users/jialiang/Project/rebotarm_ros2/src/rebotarm_twin/launch/digital_twin.launch.py


## 设备描述

### Jetson ORIN Nano 8G
1. 连接方式: `ssh damiao`  ip: 192.168.2.124 
2. 职责: 控制主从臂

### Ubuntu 22.04 ARCH AMD64
1. 连接方式: `ssh ubuntu`  ip: 192.168.55.100
2. 职责: 运行 Gazebo 仿真

### 本机 MacOS
1. 职责: 开发程序，将程序同步到 Jetson/Ubuntu


### 设备连接
1. Jetson 通过 /dev/ttyCH341USB0 串口与主臂通信
2. Jetson 通过 can0 与从臂通信
3. Ubuntu/MacOS/Jetson 都在同一局域网下
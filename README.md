#### 克隆仓库
克隆仓库并切换为ros2的humble分支(如果已经存在不需要克隆)
```bash
git clone https://github.com/agilexrobotics/piper_ros.git
git checkout humble
```

#### 环境安装
安装moveit2
```bash
sudo apt-get update
sudo apt install ros-$ROS_DISTRO-moveit
```

安装can依赖,can工具,科学计算库,Piper 机械臂SDK,ros2-control
```bash
pip3 install python-can
pip3 install scipy
pip3 install piper_sdk
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
sudo apt install can-utils
sudo apt install ethtool
```

在ros2中安装驱动串口serial_driver
```bash
sudo apt update
sudo apt install ros-humble-asio-cmake-module ros-humble-serial-driver
```

#### 构建
构建
```bash
colcon build
```

#### source
source
```bash
source install/setup.bash
```

#### 启动
can连接
```bash
bash ./pkg/piper/find_all_can_port.sh
```

配置波特率
```bash
bash ./pkg/piper/can_activate.sh can0 1000000
```

两种启动方式（前者不带rviz）
```bash
ros2 launch piper start_single_piper.launch.py 
ros2 launch piper start_single_piper_rviz.launch.py 
```

启动串口驱动
```bash
ros2 launch serial_driver serial_driver_bridge_node.launch.py
```

#### 调试
获取/end_pose
```bash
ros2 topic echo /end_pose
```
两种启动方式（前者不带rviz）
```bash
ros2 launch piper start_single_piper.launch.py 
ros2 launch piper start_single_piper_rviz.launch.py 
```
命令行控制一次机械臂，注意不要改过大，可以从/end_pose获取末端位姿
```bash
ros2 topic pub /pos_cmd piper_msgs/msg/PosCmd "{x: 0.04, y: 0.01, z: 0.3, roll: -0.15, pitch: 0.57, yaw: 0.02, gripper: 0.0}" --once
```

can连接
```bash
bash ./pkg/piper/find_all_can_port.sh
```

配置波特率
```bash
bash ./pkg/piper/can_activate.sh can0 1000000
```

获取/end_pose
```bash
ros2 topic echo /end_pose
```

两种启动方式（前者不带rviz）
```bash
ros2 launch piper start_single_piper.launch.py 
ros2 launch piper start_single_piper_rviz.launch.py 
```

命令行控制一次机械臂，注意不要改过大，可以从/end_pose获取末端位姿
```bash
ros2 topic pub /pos_cmd piper_msgs/msg/PosCmd "{x: 0.04, y: 0.01, z: 0.3, roll: -0.15, pitch: 0.57, yaw: 0.02, gripper: 0.0}" --once
```

在ros2中安装serial_driver
```bash
sudo apt update
sudo apt install ros-humble-asio-cmake-module ros-humble-serial-driver
```

启动串口驱动
```bash
ros2 launch serial_driver serial_driver_bridge_node.launch.py
```

查看serial_driver的topic
```bash
ros2 topic list
```

打印serial_read
```bash
ros2 topic echo serial_read --once
```


安装moveit2
```bash
sudo apt-get update
sudo apt install ros-$ROS_DISTRO-moveit
```

克隆仓库并切换为ros2的humble分支
```bash
git clone https://github.com/agilexrobotics/piper_ros.git
git checkout humble
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

编译
```bash
colcon build
```

source
```bash
source install/setup.bash
```
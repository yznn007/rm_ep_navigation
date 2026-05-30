# RoboMaster EP 建图与导航工作空间

## 项目简介

基于 ROS Noetic 的 DJI RoboMaster EP 自动建图（SLAM）与自主导航系统。

### 核心功能

- 🚗 **底盘驱动**：RoboMaster EP 全向麦轮驱动，发布里程计、IMU、云台关节、相机图像
- 📡 **激光雷达感知**：RPLIDAR A2 驱动，360° 激光扫描
- 🗺️ **SLAM 建图**：gmapping 实时建图，支持遥控/键盘控制遍历
- 🧭 **自主导航**：AMCL 蒙特卡洛定位 + TEB 全向局部规划 + 代价地图动态避障
- ~~🎮 **手柄遥控**：手柄控制底盘运动，按钮4启用，支持全向移动~~

## 项目文档

[详细文档](docs/details.md) — 工作空间结构、包说明、话题与坐标系、关键配置、常见故障排查

## 快速开始

### 环境要求

#### 硬件

- **上位机**：Nvidia Jetson Xavier NX
- **底盘**：DJI RoboMaster EP（序列号 `3JKDH3B001891M`）
- **雷达**：思岚 RPLIDAR A2（串口 `/dev/ttyUSB0`）

#### 软件

- **操作系统**：Ubuntu 20.04
- **ROS**：Noetic
- **Python**：3.8+
- **C++**：C++11
- **构建工具**：`catkin_make`

### 获取源码

```bash
cd ~
git clone https://github.com/yznn007/rm_ep_navigation catkin_ws
```

### 安装依赖

```bash
# 初始化 rosdep（首次使用）
sudo rosdep init && rosdep update

# ROS 包依赖（自动解析所有 package.xml）
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Python 依赖（rosdep 不可解析的 pip 包）
pip3 install robomaster
```

### 构建与环境加载

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

可选：写入 `~/.bashrc`。

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## 启动

执行前先加载环境（写入 `.bashrc` 可省略）：

```bash
source ~/catkin_ws/devel/setup.bash
```

### 底盘启动

```bash
roslaunch rm_ep_driver rm_ep_bringup.launch
```

### 键盘控制节点

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### ~~手柄遥控（未测试）~~

```bash
roslaunch rm_ep_driver teleop.launch
```

> **注意**：`teleop.launch` 默认使用 WiFi 直连模式 (`ep_conn_type:=ap`)。手柄按钮 4 启用控制。

### 建图启动

```bash
roslaunch rm_ep_navigation mapping.launch
```

### 地图保存（默认名称 `default_map`）

```bash
rosrun rm_ep_navigation save_map.sh [地图名称]
```

### 导航启动

```bash
roslaunch rm_ep_navigation navigation.launch \
  map_file:=~/catkin_ws/src/rm_ep_navigation/maps/你的地图.yaml
```

## 命令速查

```bash
# 编译
catkin_make

# 加载环境
source ~/catkin_ws/devel/setup.bash

# 删除构建产物
rm -rf build/ devel/

# 查看话题
rostopic list

# 查看节点
rosnode list

# 查看 TF 树
rosrun tf view_frames

# 雷达设备检查
ls -l /dev/ttyUSB*

# 底盘驱动
roslaunch rm_ep_driver rm_ep_bringup.launch

# 手柄遥控
roslaunch rm_ep_driver teleop.launch

# 建图
roslaunch rm_ep_navigation mapping.launch

# 导航
roslaunch rm_ep_navigation navigation.launch map_file:=/path/to/map.yaml

# 保存地图
rosrun rm_ep_navigation save_map.sh [名称]

# 键盘控制
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 调试命令
rostopic echo /odom          # 查看里程计
rostopic echo /imu           # 查看 IMU
rostopic echo /cmd_vel       # 查看速度指令
rostopic echo /scan          # 查看雷达数据
```

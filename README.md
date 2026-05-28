# RoboMaster EP 建图与导航工作空间

基于 ROS Noetic 的 DJI RoboMaster EP 自动建图（SLAM）与自主导航系统。

## 工作空间结构

```
catkin_ws/
└── src/
    ├── rplidar_ros/              思岚 RPLIDAR A2 激光雷达驱动
    ├── rm_ep_driver/             RoboMaster EP ROS 驱动节点
    ├── rm_ep_description/        EP 机器人 URDF 模型
    ├── rm_ep_navigation/         建图与导航配置包
```

## 功能包说明

### 1. rm_ep_driver — EP 驱动节点

封装 DJI RoboMaster SDK，桥接 ROS 与 EP 硬件。

| 数据 | 话题 | 方向 | 说明 |
|------|------|------|------|
| 里程计 | `/odom` | 发布 | 底盘编码器推算 |
| IMU | `/imu` | 发布 | 姿态 + 角速度 + 加速度 |
| 速度指令 | `/cmd_vel` | 订阅 | 转为 EP 全向麦轮控制 |

### 2. rm_ep_description — 机器人模型

URDF/XACRO 模型，定义 TF 树：

```
map ──(gmapping/amcl)──► odom ──(EKF)──► base_link ──┬── base ──┬── gimbal → camera
                                                      │          └── imu_link
                                                      └── laser_link
```

### 3. rm_ep_navigation — 建图与导航

| 模式 | launch 文件 | 核心算法 |
|------|------------|----------|
| 建图 | `mapping.launch` | gmapping SLAM |
| 导航 | `navigation.launch` | AMCL 定位 + TEB 全向规划 |
| 里程融合 | 内置 | robot_localization EKF（IMU + 里程计） |

## 环境依赖

### 系统要求

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Python | 3.8+ |

### ROS 包依赖

```
ros-noetic-gmapping
ros-noetic-amcl
ros-noetic-move-base
ros-noetic-map-server
ros-noetic-robot-state-publisher
ros-noetic-joint-state-publisher-gui
ros-noetic-robot-localization
ros-noetic-teb-local-planner
```

### Python 依赖

```
pip3 install robomaster
```

## 安装

### 1. 安装 ROS 依赖

```bash
sudo apt install -y \
  ros-noetic-gmapping \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-map-server \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-localization \
  ros-noetic-teb-local-planner
```

### 2. 安装 Python SDK

```bash
# 官方源
pip3 install robomaster

# 国内镜像（更快）
pip3 install robomaster -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 底盘控制

```bash
source ~/catkin_ws/devel/setup.bash

# 直接启动底盘驱动（使用配置文件中的 SN，默认 3JKDH3B001891M）
roslaunch rm_ep_driver rm_ep_bringup.launch

# 或手柄遥控
roslaunch rm_ep_driver teleop.launch
```

启动后可通过 `/cmd_vel` 话题控制底盘运动，订阅 `/odom` 和 `/imu` 获取传感器数据。

### 建图

```bash
source ~/catkin_ws/devel/setup.bash

# 直接启动建图（使用默认 SN 和 AP 直连模式）
roslaunch rm_ep_navigation mapping.launch

# 通过遥控或键盘控制 EP 遍历环境
# 地图满意后，另开终端保存:
rosrun rm_ep_navigation save_map.sh my_map
```

建图 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `ap` | 连接模式：`ap`(WiFi直连) / `sta`(路由器) / `rndis`(USB) |
| `ep_ip` | `""` | EP IP 地址 |
| `serial_port` | `/dev/ttyUSB0` | 雷达串口号 |
| `rviz` | `true` | 是否启动 RVIZ |

### 导航

```bash
source ~/catkin_ws/devel/setup.bash

# 加载地图并启动导航
roslaunch rm_ep_navigation navigation.launch \
  map_file:=/home/你的用户名/catkin_ws/src/rm_ep_navigation/maps/my_map.yaml

# 在 RVIZ 中使用 "2D Nav Goal" 工具点击目标点即可
```

导航 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `ap` | 连接模式：`ap`(WiFi直连) / `sta`(路由器) / `rndis`(USB) |
| `ep_ip` | `""` | EP IP 地址 |
| `map_file` | `.../maps/map_test.yaml` | 地图文件完整路径 |
| `serial_port` | `/dev/ttyUSB0` | 雷达串口号 |
| `rviz` | `true` | 是否启动 RVIZ |

### 地图保存脚本

```bash
# 保存为指定名称
rosrun rm_ep_navigation save_map.sh 教室地图

# 不指定名称则保存为 my_map
rosrun rm_ep_navigation save_map.sh
```

## 配置调优

所有参数配置文件位于 `rm_ep_navigation/config/`：

| 文件 | 用途 |
|------|------|
| `ekf.yaml` | 里程计 + IMU EKF 融合 |
| `gmapping_params.yaml` | gmapping SLAM 参数 |
| `amcl_params.yaml` | AMCL 定位参数 |
| `costmap_common_params.yaml` | 通用代价地图 |
| `global_costmap_params.yaml` | 全局代价地图 |
| `local_costmap_params.yaml` | 局部代价地图 |
| `teb_local_planner_params.yaml` | TEB 全向规划器 |
| `move_base_params.yaml` | move_base 框架参数 |

### 关键调参项

**速度限制** (`teb_local_planner_params.yaml`)：

```yaml
max_vel_x: 0.5       # 前向最大速度 (m/s)
max_vel_y: 0.3       # 横向最大速度 (m/s)，麦轮特有
max_vel_theta: 0.8   # 最大旋转速度 (rad/s)
```

**机器人足迹** (`costmap_common_params.yaml`)：

```yaml
footprint: [[-0.18, -0.14], [-0.18, 0.14], [0.18, 0.14], [0.18, -0.14]]
```

**障碍物安全距离** (`teb_local_planner_params.yaml`)：

```yaml
min_obstacle_dist: 0.15   # 最小障碍物距离 (m)
inflation_radius: 0.30    # 膨胀半径 (m)，在 costmap_common_params 中
```

## 硬件连接

EP 支持三种连接方式：

| 模式 | 参数 | 说明 |
|------|------|------|
| WiFi 直连 | `ep_conn_type:=ap` | 电脑直连 EP 自带 WiFi 热点 |
| 路由器 | `ep_conn_type:=sta` | EP + 电脑通过路由器通信 |
| USB | `ep_conn_type:=rndis` | USB 线直连，无需 WiFi |

RPLIDAR A2 通过 USB 连接电脑，默认串口 `/dev/ttyUSB0`。

## 常见问题

**Q: 驱动节点启动失败，提示 "RoboMaster SDK 不可用"**

```bash
pip3 install robomaster
```

**Q: EP 连接不上**

确认 EP 已开机，WiFi 直连模式需要电脑连接 EP 的热点；路由器模式需确认在同一局域网。也可尝试指定 IP：

```bash
roslaunch rm_ep_navigation mapping.launch ep_ip:=192.168.x.x
```

**Q: 雷达不工作**

```bash
# 检查串口设备
ls /dev/ttyUSB*
# 检查权限
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

**Q: 里程计漂移严重**

EP 麦轮在光滑地面容易打滑，建图时尽量低速平稳移动。EKF 融合 IMU 可部分改善，但无法完全消除。

**Q: 导航时 TEB 报错**

确认 `ros-noetic-teb-local-planner` 已安装：

```bash
dpkg -l | grep teb-local-planner
```

## RVIZ 快捷键

| 操作 | 快捷键 |
|------|--------|
| 设置初始位姿 | 工具栏 "2D Pose Estimate" |
| 设置导航目标 | 工具栏 "2D Nav Goal" |
| 旋转视角 | 鼠标左键拖拽 |
| 平移视角 | 鼠标中键拖拽 |
| 缩放 | 滚轮 |

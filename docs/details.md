# RoboMaster EP 详细文档

## 工作空间结构

```text
catkin_ws/
├── src/
│   ├── rm_ep_driver/          # EP 底盘驱动
│   │   ├── launch/            # rm_ep_chassis_bringup.launch, teleop.launch
│   │   └── scripts/           # rm_ep_chassis_driver.py
│   ├── rm_ep_navigation/      # 建图与导航
│   │   ├── launch/            # mapping.launch, navigation.launch
│   │   ├── scripts/           # save_map.sh
│   │   ├── config/            # gmapping, amcl, teb, costmap, ekf 等 YAML
│   │   ├── rviz/              # mapping_nav.rviz, nav.rviz
│   │   └── maps/              # 地图保存目录
│   ├── rm_ep_description/     # URDF 模型与 STL 网格
│   │   ├── urdf/              # rm_ep.urdf.xacro
│   │   ├── launch/            # description.launch
│   │   └── meshes/visual/     # base.stl, gimbal_*.stl
│   └── rplidar_ros/           # RPLIDAR A2 雷达 C++ 驱动
├── build/
├── devel/
└── README.md
```

## 包说明

| 包名 | 路径 | 语言 | 作用 |
|---|---|---|---|
| `rm_ep_driver` | `src/rm_ep_driver/` | Python 3.8+ | 底盘驱动，发布 `/odom`、`/imu`，订阅 `/cmd_vel` |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 纯配置 | 建图(gmapping)、导航(AMCL+TEB)、EKF 融合 |
| `rm_ep_description` | `src/rm_ep_description/` | 纯配置 | URDF 模型与 STL 网格，`robot_state_publisher` 发布静态 TF |
| `rplidar_ros` | `src/rplidar_ros/` | C++ (C++11) | 思岚 RPLIDAR A2 激光雷达驱动节点，自带 SDK 源码编译 |

## 话题与坐标系

### 主要话题

**rm_ep_driver**：

- 订阅：`/cmd_vel`（`geometry_msgs/Twist`）— 速度指令
- 发布：`/odom`（`nav_msgs/Odometry`）— 里程计，frame_id=`odom`，child=`base_link`
- 发布：`/imu`（`sensor_msgs/Imu`）— IMU 数据，frame_id=`imu_link`

**rplidar_ros**：

- 发布：`/scan`（`sensor_msgs/LaserScan`）— 激光雷达扫描数据

### `/cmd_vel` 数据流

```
move_base / teleop → /cmd_vel → 驱动节点 → SDK drive_speed
```

驱动节点内部做坐标变换：`x=msg.linear.x, y=-msg.linear.y, z=-deg(msg.angular.z)`

### 常用坐标系

- `map` — 地图坐标系（gmapping/amcl 发布）
- `odom` — 里程计坐标系（EKF 发布 odom→base_link TF）
- `base_link` — 机器人基座坐标系
- `laser_link` — 激光雷达坐标系（URDF fixed 关节）
- `imu_link` — IMU 坐标系（在旋转中心）

### TF 树

```
map ──(gmapping/amcl)──► odom ──(EKF)──► base_link ──(URDF)──► laser_link
                                                                  ├── imu_link
                                                                  ├── chassis_base_link
                                                                  │   └── arm → camera
                                                                  └── wheels (4个麦轮)
```

**重要**：底盘驱动不发布 TF，由 EKF 统一发布 `odom→base_link`。

## 关键配置

### 底盘驱动

新驱动 `rm_ep_chassis_driver.py` 从 ROS2 移植，参数通过 launch 文件传递。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式：`rndis`(USB) / `ap`(WiFi直连) / `sta`(路由器) |
| `ep_ip` | `""` | EP IP 地址（留空则通过 SN 自动发现） |
| `odom_rate` | 20 | 里程计发布频率 (Hz) |
| `cmd_vel_timeout` | 0.5 | 速度指令超时，自动停车 (秒) |
| `enable_cmd_vel` | true | 订阅 `/cmd_vel` |
| `imu_has_orientation` | true | IMU 消息包含姿态 |

### EKF 融合

EKF 融合策略（`ekf.yaml`）：

- **odom**：绝对位置 X,Y + 世界坐标系速度 vx,vy + 角速度 vyaw
- **IMU**：绝对 Yaw 角 + 角速度 vyaw + 加速度 ax,ay
- `imu0_relative: true`：上电瞬间 yaw 视为 0 度

### 建图（gmapping）参数

参数文件：`src/rm_ep_navigation/config/gmapping_params.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `particles` | 30 | 粒子数量 |
| `delta` | 0.05 | 地图分辨率 (m/像素) |
| `maxUrange` | 8.0 | 激光雷达最大可用距离 (m) |
| `linearUpdate` | 0.1 | 平移 10cm 触发扫描处理 |
| `angularUpdate` | 0.05 | 旋转 2.87° 触发扫描处理 |

### 定位（AMCL）参数

参数文件：`src/rm_ep_navigation/config/amcl_params.yaml`

- 粒子数：100 ~ 2000（自适应）
- 激光模型：`likelihood_field`

### 导航（move_base）参数

**代价地图** (`costmap_common_params.yaml`)：

- 机器人足迹：36cm × 28cm（矩形）
- 膨胀半径：0.30m

**全局代价地图** (`global_costmap_params.yaml`)：

- 尺寸：20m × 20m，更新频率 2Hz

**局部代价地图** (`local_costmap_params.yaml`)：

- 尺寸：4m × 4m，更新频率 5Hz（滚动窗口）

**TEB 局部规划器** (`teb_local_planner_params.yaml`)：

- 最大速度：vx=0.8 m/s, vy=0.0 m/s, vθ=1.0 rad/s
- 全向运动学模型
- 最小障碍物距离：0.15m

### 连接模式说明

EP 支持三种连接方式：

| 模式 | 参数 | 说明 |
|------|------|------|
| USB | `ep_conn_type:=rndis` | USB 线直连，无需 WiFi，**默认模式** |
| WiFi 直连 | `ep_conn_type:=ap` | 电脑连接 EP 自带 WiFi 热点 |
| 路由器 | `ep_conn_type:=sta` | EP + 电脑连接同一路由器 |

### 雷达参数

启动时通过 launch 文件参数配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | 串口设备 |
| `serial_baudrate` | `256000` | 波特率 |
| `frame_id` | `laser_link` | TF 帧名 |
| `inverted` | `false` | 是否反转角度 |
| `angle_compensate` | `true` | 角度补偿 |

## SDK 坐标系注意事项

RoboMaster SDK 坐标系与 ROS REP-103 标准的差异：

- **y 轴方向相反**：SDK y 正=右，ROS y 正=左
- **yaw 方向相反**：SDK 顺时针正，ROS 逆时针正

驱动中的映射（与 ROS2 一致）：

| 数据 | 映射 |
|------|------|
| 位置 | `x=px, y=-py` |
| 速度 | `vx=vgx, vy=-vgy`（世界坐标系） |
| 姿态 | `yaw=-yaw_deg, pitch=-pitch_deg, roll=roll_deg` |
| IMU | `acc_y=-acc_y, acc_z=-acc_z, gyro_y=-gyro_y, gyro_z=-gyro_z` |
| cmd_vel | `x=x, y=-y, z=-z` |

**修改任何坐标映射时必须保持 odom 和 cmd_vel 一致。**

### SDK 连接问题

RoboMaster SDK 使用 `is` 比较字符串（不是 `==`），驱动必须使用 SDK 常量对象：

```python
from robomaster import conn as rm_conn
conn_type_map = {
    'ap': rm_conn.CONNECTION_WIFI_AP,
    'sta': rm_conn.CONNECTION_WIFI_STA,
    'rndis': rm_conn.CONNECTION_USB_RNDIS,
}
conn_type = conn_type_map.get(self.ep_conn_type, self.ep_conn_type)
```

## 常见故障排查

### 编译失败

```bash
# 确认依赖已安装
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 清理后重新编译
rm -rf build/ devel/
catkin_make
```

### EP 连接失败

- 确认 EP 已开机
- 检查 SN 号是否正确
- USB 模式确认 USB 线已连接
- WiFi 直连模式确认电脑已连接 EP 热点
- 可尝试指定 IP：`ep_ip:=192.168.x.x`

### 雷达不工作

```bash
# 检查串口设备是否存在
ls -l /dev/ttyUSB*
# 检查权限
groups  # 确认是否在 dialout 组
# 如未加入：
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 驱动节点启动失败，提示 SDK 不可用

```bash
pip3 install robomaster
```

### robot_state_publisher 崩溃

检查 URDF 中是否有重复的材质定义。如果使用 ROS2 移植的 URDF，确保每个材质只定义一次。

### 导航时 TEB 报错

确认依赖已安装：

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 里程计漂移严重

EP 麦轮在光滑地面容易打滑，建图时尽量低速平稳移动。EKF 融合 IMU 可减少累积误差。

### TF 树异常

```bash
# 检查当前 TF 树
rosrun tf view_frames
# 查看具体两个帧之间的变换
rosrun tf tf_echo odom base_link
```

# RoboMaster EP 详细文档

## 工作空间结构

```text
catkin_ws/
├── src/
│   ├── rm_ep_driver/          # EP 底盘驱动
│   │   ├── launch/            # rm_ep_bringup.launch, teleop.launch, cmd_vel_remap.launch
│   │   ├── scripts/           # rm_ep_driver_node.py, cmd_vel_remap.py
│   │   └── config/            # rm_ep_params.yaml
│   ├── rm_ep_navigation/      # 建图与导航
│   │   ├── launch/            # mapping.launch, navigation.launch
│   │   ├── scripts/           # save_map.sh
│   │   ├── config/            # gmapping, amcl, teb, costmap, ekf 等 8 个 YAML
│   │   ├── rviz/              # mapping_nav.rviz
│   │   └── maps/              # 地图保存目录
│   ├── rm_ep_description/     # URDF 模型与 STL 网格
│   │   ├── urdf/              # rm_ep.urdf.xacro
│   │   ├── launch/            # description.launch, display.launch
│   │   └── meshes/visual/     # base.stl, gimbal_*.stl
│   └── rplidar_ros/           # RPLIDAR A2 雷达 C++ 驱动
├── build/
├── devel/
└── README.md
```

## 包说明

| 包名 | 路径 | 语言 | 作用 |
|---|---|---|---|
| `rm_ep_driver` | `src/rm_ep_driver/` | Python 3.8+ | 底盘串口/网口驱动，发布 `/odom`、`/imu`、`/joint_states`，订阅 `/cmd_vel_rm_ep` |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 纯配置 | 建图(gmapping)、导航(AMCL+TEB)、EKF 融合(已禁用) |
| `rm_ep_description` | `src/rm_ep_description/` | 纯配置 | URDF 模型与 STL 网格，`robot_state_publisher` 发布静态 TF |
| `rplidar_ros` | `src/rplidar_ros/` | C++ (C++11) | 思岚 RPLIDAR A2 激光雷达驱动节点，自带 SDK 源码编译 |

## 话题与坐标系

### 主要话题

**rm_ep_driver**：

- 订阅：`/cmd_vel_rm_ep`（`geometry_msgs/Twist`）— 由 `cmd_vel_remap` 桥接自 `/cmd_vel`
- 发布：`/odom`（`nav_msgs/Odometry`）— 里程计，frame_id=`odom`，child=`base_link`
- 发布：`/imu`（`sensor_msgs/Imu`）— IMU 数据，frame_id=`imu_link`
- 发布：`/joint_states`（`sensor_msgs/JointState`）— 云台关节角度
- 发布：`/camera/image_raw`（`sensor_msgs/Image`）— 相机图像流

**rplidar_ros**：

- 发布：`/scan`（`sensor_msgs/LaserScan`）— 激光雷达扫描数据

### `/cmd_vel` 数据流

```
move_base / teleop → /cmd_vel → [cmd_vel_remap: x↔y swap] → /cmd_vel_rm_ep → 驱动节点
```

`rm_ep_bringup.launch` 默认启动 `cmd_vel_remap` 桥接节点。该节点做 xy 交换，驱动节点再做第二次 x↔y 映射，两次抵消后净效果为直通。这是历史遗留设计，暂不修改。

### 常用坐标系

- `map` — 地图坐标系（gmapping/amcl 发布）
- `odom` — 里程计坐标系（驱动直接发布 odom→base_link TF）
- `base_link` — 机器人基座坐标系
- `laser_link` — 激光雷达坐标系（URDF fixed 关节）
- `imu_link` — IMU 坐标系

### TF 树（当前实际）

```
map ──(gmapping/amcl)──► odom ──(rm_ep_driver 直接发布 TF)──► base_link
                                                                  ├── laser_link (URDF fixed)
                                                                  ├── imu_link
                                                                  └── gimbal → camera
```

> **注意**：EKF 配置文件 `ekf.yaml` 仍存在，但在 `mapping.launch` 和 `navigation.launch` 中均已注释掉，原因是驱动直接发布 `odom→base_link` TF，同时启用 EKF 会造成双源 TF 冲突。如需重新启用 EKF，必须同时注释掉驱动中 `_publish_odometry` 方法末尾的 `_tf_broadcaster.sendTransform` 调用。

## 关键配置

### 底盘与 EP 驱动

驱动参数文件：`src/rm_ep_driver/config/rm_ep_params.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `"3JKDH3B001891M"` | EP 序列号 |
| `ep_conn_type` | `"rndis"` | 连接模式：`rndis`(USB) / `ap`(WiFi直连) / `sta`(路由器) |
| `ep_ip` | `""` | EP IP 地址（留空则通过 SN 自动发现） |
| `odom_rate` | 20 | 里程计发布频率 (Hz) |
| `imu_rate` | 20 | IMU 发布频率 (Hz) |
| `cmd_vel_timeout` | 0.5 | 速度指令超时，自动停车 (秒) |
| `enable_cmd_vel` | true | 订阅 `/cmd_vel_rm_ep` |
| `enable_camera` | true | 发布 `/camera/image_raw` |
| `enable_gimbal` | true | 发布 `/joint_states` |
| `gimbal_rate` | 50 | 云台数据订阅频率 (Hz) |
| `init_attitude_calibration` | true | 用初始姿态校准里程计（输出相对位姿） |
| `imu_gravity_constant` | 9.86 | 重力常数，>0 则将加速度乘以此值转为 m/s² |
| `imu_flip_x` / `imu_flip_y` | false | 修正 IMU 轴与 URDF 之间的方向差异 |
| `yaw_offset_deg` | 0.0 | yaw 偏移修正角度（正值=逆时针） |

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

- 最大速度：vx=0.5 m/s, vy=0.3 m/s, vθ=0.8 rad/s
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

RoboMaster SDK 的 yaw 旋转方向与 ROS REP-103 标准相反：

- SDK：顺时针为正 (CW+)
- ROS：逆时针为正 (CCW+)

驱动节点在以下三处统一做了取反（修改时必须保持三处一致）：

- `_cmd_vel_callback`: `vz_rad = -msg.angular.z`
- `_publish_odometry`: `angular.z = -math.radians(vz)`，以及姿态四元数 Z 取反
- `_publish_imu`: `angular_velocity.z = -math.radians(gyro_z)`

同时 SDK x/y 轴与 ROS 存在交换：SDK `(x, y)` = ROS `(y, x)`。驱动在 `_publish_odometry` 中做了 `position.x = py; position.y = px`。

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

### 导航时 TEB 报错

确认依赖已安装：

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 里程计漂移严重

EP 麦轮在光滑地面容易打滑，建图时尽量低速平稳移动。初始姿态校准 (`init_attitude_calibration: true`) 可减少累积误差。

### TF 树异常

```bash
# 检查当前 TF 树
rosrun tf view_frames
# 查看具体两个帧之间的变换
rosrun tf tf_echo odom base_link
```

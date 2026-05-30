# AGENTS.md — RoboMaster EP 建图与导航工作空间

## 工作空间

ROS Noetic catkin 工作空间。DJI RoboMaster EP + RPLIDAR A2 自动建图与导航。

无测试、无 lint、无 CI。构建产物 (`build/`、`devel/`) 已在 `.gitignore` 排除。

## 关键命令

```bash
# 编译（工作空间根目录）
catkin_make

# 每个新终端必须 source
source ~/catkin_ws/devel/setup.bash

# 启动底盘驱动（默认 ep_conn_type=rndis，即 USB 连接）
roslaunch rm_ep_driver rm_ep_bringup.launch

# 手柄遥控（注意 teleop.launch 默认 ep_conn_type=ap，即 WiFi 直连）
roslaunch rm_ep_driver teleop.launch

# 建图（默认 rndis）
roslaunch rm_ep_navigation mapping.launch

# 导航
roslaunch rm_ep_navigation navigation.launch map_file:=/home/xxx/catkin_ws/src/rm_ep_navigation/maps/my_map.yaml

# 保存地图（默认名称 default_map）
rosrun rm_ep_navigation save_map.sh [名称]
```

### Launch 参数（mapping / navigation 通用）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式: `rndis`(USB) / `ap`(WiFi直连) / `sta`(路由器) |
| `ep_ip` | `""` | EP IP 地址（留空则通过 SN 自动发现） |
| `serial_port` | `/dev/ttyUSB0` | 雷达串口 |
| `rviz` | `true` | 启动 RVIZ |
| `map_file` | `...maps/default_map.yaml` | (仅 navigation) 地图文件绝对路径 |

### teleop.launch 特殊说明

`teleop.launch` 的 `ep_conn_type` 默认值是 **`ap`**（WiFi 直连），与其他 launch 文件不同。手柄按钮 4 启用控制，轴 1/0/3 分别控制 X/Y/Yaw。

## 包结构

| 包 | 路径 | 职责 |
|---|---|---|
| `rm_ep_driver` | `src/rm_ep_driver/` | EP 底盘驱动，发布 `/odom`、`/imu`、`/joint_states`，订阅 `/cmd_vel_rm_ep` |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 建图(gmapping)、导航(AMCL+TEB)、EKF 配置（当前未启用） |
| `rm_ep_description` | `src/rm_ep_description/` | URDF 模型，`robot_state_publisher` 发布静态 TF |
| `rplidar_ros` | `src/rplidar_ros/` | RPLIDAR A2 C++ 驱动 |

## `/cmd_vel` 数据流（关键）

move_base / teleop 发布 **`/cmd_vel`**，但驱动节点实际订阅 **`/cmd_vel_rm_ep`**。中间通过 `cmd_vel_remap.launch` 桥接：

```
move_base / teleop → /cmd_vel → [cmd_vel_remap: x↔y swap] → /cmd_vel_rm_ep → 驱动节点
```

`rm_ep_bringup.launch` 默认 `enable_cmd_vel_remap:=true`，自动启动桥接节点。驱动节点对 `/cmd_vel_rm_ep` 做了 **第二次** x↔y 映射（`x=vy, y=vx`），两次交换抵消后净效果为直通。

## 启动顺序

### mapping.launch
1. `rm_ep_description` → URDF + `robot_state_publisher`
2. `rplidar_ros` → 激光雷达
3. `rm_ep_driver` → 底盘驱动（包含 `cmd_vel_remap` 桥接）
4. `robot_localization` → EKF 融合（`ekf.yaml`），发布 `odom→base_link` TF
5. `gmapping` → SLAM 建图
6. `rviz`（可选）

### navigation.launch
1-3. 同上
4. `map_server` → 加载预建地图
5. `amcl` → 蒙特卡洛定位
6. `move_base` → TEB 全向规划 + costmap
7. `rviz`（可选）

## TF 树（当前实际）

```
map ──(gmapping / amcl)──► odom ──(EKF)──► base_link
                                             ├── laser_link (URDF fixed)
                                             ├── imu_link
                                             └── gimbal → camera
```

驱动节点已禁用 `odom→base_link` TF 广播，改由 EKF 融合 odom + IMU 后统一发布。如需重新禁用 EKF，需同时取消注释驱动中 `_publish_odometry` 末尾的 TF 广播代码。

## 驱动配置 (`rm_ep_params.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `"3JKDH3B001891M"` | EP 序列号 |
| `ep_conn_type` | `"rndis"` | 连接模式 |
| `odom_rate` | 20 | 里程计频率 (Hz) |
| `imu_rate` | 20 | IMU 频率 (Hz) |
| `cmd_vel_timeout` | 0.5 | 超时自动停车 (秒) |
| `enable_cmd_vel` | true | 订阅 `/cmd_vel_rm_ep` |
| `enable_camera` | true | 发布 `/camera/image_raw` |
| `enable_gimbal` | true | 发布 `/joint_states` |
| `init_attitude_calibration` | true | 初始姿态校准（输出相对位姿） |
| `imu_gravity_constant` | 9.86 | 加速度补偿 |
| `imu_flip_x` / `imu_flip_y` | false | IMU 轴翻转 |
| `yaw_offset_deg` | 0.0 | yaw 偏移修正 |

## SDK 坐标系（不会再有人告诉你的坑）

RoboMaster SDK yaw 旋转方向与 ROS REP-103 **相反**（SDK 顺时针正，ROS 逆时针正）。驱动在以下位置做了取反：

- `_cmd_vel_callback`: `vz_rad = -msg.angular.z`
- `_publish_odometry`: 姿态四元数 Z 轴取反，线速度 `angular.z = -math.radians(vz)`
- `_publish_imu`: `angular_velocity.z = -math.radians(gyro_z)`

同时，SDK x/y 轴与 ROS 有交换：SDK `(x, y)` = ROS `(y, x)`。驱动中 `_publish_odometry` 做了 `position.x = py; position.y = px`，速度也做了对应旋转。**修改任何坐标映射时必须保持这三处一致。**

## 重要入口

- 驱动节点: `src/rm_ep_driver/scripts/rm_ep_driver_node.py` — `RmEpDriver` 类
- cmd_vel 桥接: `src/rm_ep_driver/scripts/cmd_vel_remap.py`
- 建图配置: `src/rm_ep_navigation/config/gmapping_params.yaml`（30 粒子，0.05m 分辨率）
- 导航配置: `amcl_params.yaml` + `teb_local_planner_params.yaml` + costmap 系列
- 地图保存: `src/rm_ep_navigation/scripts/save_map.sh`
- 地图目录: `src/rm_ep_navigation/maps/`

## 常见问题

- **SDK 未安装**: `pip3 install robomaster`
- **串口权限**: `sudo usermod -a -G dialout $USER` 后重新登录
- **EP 连接失败**: 检查 SN，或指定 IP `ep_ip:=192.168.x.x`
- **连接模式**: 默认 USB (`rndis`)，WiFi 直连用 `ep_conn_type:=ap`，路由器用 `sta`
- **里程计漂移**: EP 麦轮在光滑地面易打滑，建图时低速平稳移动

## Git 提交规范

Conventional Commits，subject 中文动词开头，不超过 50 字符。类型: `feat` / `fix` / `perf` / `docs` / `refactor` / `style` / `test` / `chore`。scope 如 `launch`、`driver`、`amcl`。

## 环境依赖

- Ubuntu 20.04 + ROS Noetic
- ROS 包: `gmapping` `amcl` `move-base` `map-server` `robot-state-publisher` `robot-localization` `teb-local-planner`
- Python: `robomaster` `cv_bridge`
- 所有 Python 脚本使用 `#!/usr/bin/env python3`

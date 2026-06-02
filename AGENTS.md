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

# 启动底盘驱动（默认 rndis，即 USB 连接）
roslaunch rm_ep_driver rm_ep_chassis_bringup.launch

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
| `rm_ep_driver` | `src/rm_ep_driver/` | EP 底盘驱动，发布 `/odom`、`/imu`，订阅 `/cmd_vel` |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 建图(gmapping)、导航(AMCL+TEB)、EKF 融合 |
| `rm_ep_description` | `src/rm_ep_description/` | URDF 模型，`robot_state_publisher` 发布静态 TF |
| `rplidar_ros` | `src/rplidar_ros/` | RPLIDAR A2 C++ 驱动 |

## `/cmd_vel` 数据流（关键）

move_base / teleop 发布 **`/cmd_vel`**，驱动节点直接订阅 **`/cmd_vel`**：

```
move_base / teleop → /cmd_vel → 驱动节点 → SDK drive_speed
```

驱动节点内部做坐标变换：`x=msg.linear.x, y=-msg.linear.y, z=-deg(msg.angular.z)`

## 启动顺序

### mapping.launch
1. `rm_ep_description` → URDF + `robot_state_publisher`
2. `rplidar_ros` → 激光雷达
3. `rm_ep_driver` → 底盘驱动
4. `robot_localization` EKF → 融合 odom + IMU，发布 `odom→base_link` TF
5. `gmapping` → SLAM 建图，发布 `map→odom` TF
6. `rviz`（可选）

### navigation.launch
1-3. 同上
4. `map_server` → 加载预建地图
5. `amcl` → 蒙特卡洛定位
6. `move_base` → TEB 全向规划 + costmap
7. `rviz`（可选）

## TF 树

```
map ──(gmapping / amcl)──► odom ──(EKF)──► base_link ──(URDF)──► laser_link
                                                                  ├── imu_link
                                                                  ├── chassis_base_link
                                                                  │   └── arm → camera
                                                                  └── wheels (4个麦轮)
```

**重要**：底盘驱动不发布 TF，由 EKF 统一发布 `odom→base_link`。

## 坐标系映射（SDK → ROS）

SDK 坐标系与 ROS 坐标系差异：
- **y 轴方向相反**：SDK y 正=右，ROS y 正=左
- **yaw 方向相反**：SDK 顺时针正，ROS 逆时针正

驱动中的映射（与 ROS2 一致）：
- 位置：`x=px, y=-py`
- 速度：`vx=vgx, vy=-vgy`（世界坐标系）
- 姿态：`yaw=-yaw_deg, pitch=-pitch_deg, roll=roll_deg`
- IMU：`acc_y=-acc_y, acc_z=-acc_z, gyro_y=-gyro_y, gyro_z=-gyro_z`
- cmd_vel：`x=x, y=-y, z=-z`

**修改任何坐标映射时必须保持 odom 和 cmd_vel 一致。**

## EKF 配置

EKF 融合策略（`ekf.yaml`）：
- **odom**：绝对位置 X,Y + 世界坐标系速度 vx,vy + 角速度 vyaw
- **IMU**：绝对 Yaw 角 + 角速度 vyaw + 加速度 ax,ay
- `imu0_relative: true`：上电瞬间 yaw 视为 0 度

## 驱动配置

新驱动 `rm_ep_chassis_driver.py` 从 ROS2 移植，参数通过 launch 文件传递，不使用独立的 yaml 配置文件。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `3JKDH3B001891M` | EP 序列号 |
| `ep_conn_type` | `rndis` | 连接模式 |
| `odom_rate` | 20 | 里程计频率 (Hz) |
| `cmd_vel_timeout` | 0.5 | 超时自动停车 (秒) |
| `enable_cmd_vel` | true | 订阅 `/cmd_vel` |
| `imu_has_orientation` | true | IMU 消息包含姿态 |

## SDK 坐标系坑

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

## 重要入口

- 驱动节点: `src/rm_ep_driver/scripts/rm_ep_chassis_driver.py` — `RmEpChassisDriver` 类
- 底盘 bringup: `src/rm_ep_driver/launch/rm_ep_chassis_bringup.launch`
- URDF: `src/rm_ep_description/urdf/rm_ep.urdf.xacro`
- 建图配置: `src/rm_ep_navigation/config/gmapping_params.yaml`（30 粒子，0.05m 分辨率）
- 导航配置: `amcl_params.yaml` + `teb_local_planner_params.yaml` + costmap 系列
- EKF 配置: `src/rm_ep_navigation/config/ekf.yaml`
- 地图保存: `src/rm_ep_navigation/scripts/save_map.sh`
- 地图目录: `src/rm_ep_navigation/maps/`

## 常见问题

- **SDK 未安装**: `pip3 install robomaster`
- **串口权限**: `sudo usermod -a -G dialout $USER` 后重新登录
- **EP 连接失败**: 检查 SN，或指定 IP `ep_ip:=192.168.x.x`
- **连接模式**: 默认 USB (`rndis`)，WiFi 直连用 `ep_conn_type:=ap`，路由器用 `sta`
- **里程计漂移**: EP 麦轮在光滑地面易打滑，建图时低速平稳移动
- **robot_state_publisher 崩溃**: 检查 URDF 中是否有重复的材质定义

## Git 提交规范

Conventional Commits，subject 中文动词开头，不超过 50 字符。类型: `feat` / `fix` / `perf` / `docs` / `refactor` / `style` / `test` / `chore`。scope 如 `launch`、`driver`、`amcl`、`description`。

## 环境依赖

- Ubuntu 20.04 + ROS Noetic
- ROS 包: `gmapping` `amcl` `move-base` `map-server` `robot-state-publisher` `robot-localization` `teb-local-planner`
- Python: `robomaster` `cv_bridge` `defusedxml`
- 所有 Python 脚本使用 `#!/usr/bin/env python3`

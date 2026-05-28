# AGENTS.md - RoboMaster EP 建图与导航工作空间

## 工作空间类型
ROS Noetic catkin 工作空间，用于 DJI RoboMaster EP 自动建图与导航。

## 设备 SN 码

当前 EP 序列号：**`3JKDH3B001891M`**（配置在 `src/rm_ep_driver/config/rm_ep_params.yaml`）

## 关键命令

```bash
# 编译（必须在工作空间根目录执行）
catkin_make

# 每次新终端必须 source
source ~/catkin_ws/devel/setup.bash

# 启动底盘驱动
roslaunch rm_ep_driver rm_ep_bringup.launch ep_sn:=YOUR_SN

# 手柄遥控（底盘 + joy + teleop）
roslaunch rm_ep_driver teleop.launch ep_sn:=YOUR_SN

# 启动建图（需要指定 EP SN）
roslaunch rm_ep_navigation mapping.launch ep_sn:=YOUR_SN

# 启动导航
roslaunch rm_ep_navigation navigation.launch ep_sn:=YOUR_SN map_file:=/path/to/map.yaml

# 保存地图
rosrun rm_ep_navigation save_map.sh [地图名称]
```

## 包结构与职责

| 包 | 路径 | 说明 |
|---|---|---|
| `rm_ep_driver` | `src/rm_ep_driver/` | EP 底盘驱动，发布 `/odom`、`/imu`，订阅 `/cmd_vel` |
| `rm_ep_navigation` | `src/rm_ep_navigation/` | 建图(gmapping)、导航(AMCL+TEB)、EKF融合 |
| `rm_ep_description` | `src/rm_ep_description/` | URDF 模型，定义 TF 树 |
| `rplidar_ros` | `src/rplidar_ros/` | RPLIDAR A2 激光雷达驱动 |
| `catkin_simple` | `src/catkin_simple/` | catkin cmake 辅助工具 |

## 入口点

- **驱动主节点**: `src/rm_ep_driver/scripts/rm_ep_driver_node.py` — `RmEpDriver` 类
- **底盘 launch**: `src/rm_ep_driver/launch/rm_ep_bringup.launch`
- **手柄 launch**: `src/rm_ep_driver/launch/teleop.launch`
- **建图 launch**: `src/rm_ep_navigation/launch/mapping.launch`
- **导航 launch**: `src/rm_ep_navigation/launch/navigation.launch`

### mapping.launch 启动顺序
1. `rm_ep_description` → 加载 URDF + `robot_state_publisher`
2. `rplidar_ros` → 激光雷达驱动
3. `rm_ep_driver` → 底盘驱动
4. `robot_localization` → EKF 融合（`ekf.yaml`）
5. `gmapping` → SLAM 建图（`gmapping_params.yaml`）
6. `rviz` → 可视化（可选）

### navigation.launch 启动顺序
1-4. 同上
5. `map_server` → 加载预建地图
6. `amcl` → 蒙特卡洛定位（`amcl_params.yaml`）
7. `move_base` → 导航框架（TEB 局部规划器 + costmap）
8. `rviz` → 可视化（可选）

## 配置文件

- `src/rm_ep_driver/config/rm_ep_params.yaml` — EP 连接参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ep_sn` | `"3JKDH3B001891M"` | EP 序列号 |
| `ep_conn_type` | `"ap"` | 连接模式（ap=WiFi直连 / sta=路由器 / rndis=USB） |
| `odom_rate` | 20 Hz | 里程计发布频率 |
| `imu_rate` | 20 Hz | IMU 发布频率 |
| `cmd_vel_timeout` | 0.5 s | 超时自动停车 |
| `enable_camera` | true | 启用相机流 |
| `enable_gimbal` | true | 启用云台关节 |
| `init_attitude_calibration` | true | 初始姿态校准 |
| `imu_gravity_constant` | 9.86 | 重力补偿 |

- `src/rm_ep_navigation/config/` — 导航参数：
  - `ekf.yaml` — EKF 融合（30Hz，融合 odom + IMU，发布 odom→base_link TF）
  - `gmapping_params.yaml` — 粒子数30，地图分辨率0.05m
  - `amcl_params.yaml` — 粒子100~2000，likelihood_field 模型
  - `costmap_common_params.yaml` — 足迹 32cm×28cm，膨胀半径0.30m
  - `global_costmap_params.yaml` — 20m×20m，2Hz
  - `local_costmap_params.yaml` — 4m×4m，5Hz，滚动窗口
  - `teb_local_planner_params.yaml` — 全向，vx=0.5, vy=0.3, vtheta=0.8
  - `move_base_params.yaml` — 控制器10Hz，规划器1Hz

## 硬件依赖

- DJI RoboMaster EP（需安装 `pip3 install robomaster`）
- RPLIDAR A2 激光雷达（串口 `/dev/ttyUSB0`）
- 手柄（可选，通过 `teleop.launch` 启动，按钮4启用，轴1/0/3控制）

## 驱动节点内部逻辑

`RmEpDriver` 类 (`rm_ep_driver_node.py`):

**数据流**:
- `chassis.sub_position` + `sub_attitude` + `sub_velocity` → `/odom`（frame_id=`odom`, child=`base_link`）
- `chassis.sub_imu` → `/imu`（frame_id=`imu_link`）
- 欧拉角→四元数转换（ZYX 顺序），初始朝向校准

**控制流**:
- 订阅 `/cmd_vel`（`geometry_msgs/Twist`）→ `chassis.drive_speed(x, y, z_deg)`
- `cmd_vel_timeout` 超时自动停车

## TF 树结构

```
map ──(gmapping/amcl)──► odom ──(EKF)──► base_link ──┬── base (底盘STL)
                                                      │    ├── gimbal_yaw_joint → gimbal_base → gimbal_pitch_joint → gimbal_head → camera_link → camera_link_optical_frame
                                                      │    └── imu_link
                                                      └── laser_link (URDF fixed 关节，robot_state_publisher 发布)
```

## 源目录结构

```
src/
├── rm_ep_driver/
│   ├── launch/rm_ep_bringup.launch, teleop.launch
│   ├── scripts/rm_ep_driver_node.py
│   └── config/rm_ep_params.yaml
├── rm_ep_navigation/
│   ├── launch/mapping.launch, navigation.launch
│   ├── scripts/save_map.sh
│   ├── config/ (8个 YAML)
│   ├── rviz/mapping_nav.rviz
│   └── maps/ (.gitkeep)
├── rm_ep_description/
│   ├── urdf/rm_ep.urdf.xacro
│   ├── launch/description.launch, display.launch
│   └── meshes/visual/ (base.stl, gimbal_*.stl)
├── rplidar_ros/ (C++ 驱动 + SDK)
└── catkin_simple/
```

## 重要约定

- 所有 Python 脚本使用 `#!/usr/bin/env python3`
- 无测试框架、无 lint 配置、无 CI 流程
- 构建产物 (`build/`, `devel/`) 已在 `.gitignore` 排除
- 地图文件保存在 `src/rm_ep_navigation/maps/`
- 许可证：MIT

## 常见问题

- **SDK 未安装**: `pip3 install robomaster`
- **串口权限**: `sudo usermod -a -G dialout $USER` 后重新登录
- **EP 连接失败**: 检查 SN 号，或尝试指定 IP `ep_ip:=192.168.x.x`
- **里程计漂移**: EP 麦轮在光滑地面易打滑，建图时保持低速平稳移动

## Git 提交规范

使用 Conventional Commits 格式：

```
<type>[(scope)]: <subject>

[body]
```

### Type 类型

| Type | 说明 |
|------|------|
| `feat` | 新功能 |
| `fix` | 修复 bug 或参数错误 |
| `perf` | 性能优化、参数调优 |
| `docs` | 文档变更 |
| `refactor` | 代码重构（不改变行为） |
| `style` | 代码格式（不影响运行） |
| `test` | 测试相关 |
| `chore` | 杂项（构建、配置、依赖） |

### 规则

- `subject` 使用中文，动词开头，不超过 50 字符
- `scope` 可选，标识影响范围（如 `launch`、`amcl`、`driver`）
- `body` 分行列出具体变更，每行 72 字符内
- `subject` 结尾不加句号
- 破坏性变更在 body 加入 `BREAKING CHANGE:` 说明

### 示例

```
fix(launch): 统一 EP 连接参数默认值，修正雷达波特率

将 4 个 launch 文件的 ep_sn 默认值统一为 3JKDH3B001891M
统一 ep_conn_type 默认为 ap，新增 rndis(USB) 连接模式注释
teleop.launch 补全 ep_ip 参数传递
修正 RPLIDAR A2 串口波特率 115200 → 256000
修复 rm_ep_driver_node.py 代码默认值与 YAML 对齐
```

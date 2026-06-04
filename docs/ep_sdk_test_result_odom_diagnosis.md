# RoboMaster EP SDK 测试结果与 Odom 问题诊断方案

本文基于 `test_results.md` 中的 SDK 原始数据，分析 RoboMaster EP 在静止、前进、后退、原地旋转时的 `sub_position`、`sub_velocity`、`sub_attitude`、`sub_imu` 和 `sub_status` 返回值，并给出 ROS `/odom` 的修正方案。

测试文件：

```text
F:/wechat/xwechat_files/wxid_2rrsagwnm33y22_006b/temp/RWTemp/2026-06/e341d466b6aaef2f2cba5569da0a3dcd/test_results.md
```

## 1. 总体结论

这次测试结果不支持“EP SDK 原始 odom 硬件严重漂移”的判断。

更准确的结论是：

```text
EP SDK 原始 position、velocity、attitude、imu 数据整体正常。
当前 ROS odom 问题更可能来自：
1. 启动时没有做 odom 原点归零。
2. 没有扣除初始 yaw。
3. 没有把 EP 坐标旋转到 ROS odom 初始坐标系。
4. yaw 度/弧度处理可能错误。
5. cmd_vel 与 odom 的坐标符号转换不一致。
6. odom.twist 速度来源可能使用不当。
7. 可能存在重复发布 odom -> base_link TF。
```

所以当前不应该优先更换硬件或放弃 EP `sub_position`，而应该先修正 ROS `/odom` 发布逻辑。

## 2. 测试数据摘要

| 测试 | position.x 变化 | position.y 变化 | yaw 变化 | slip_flag | 判断 |
|---|---:|---:|---:|---:|---|
| 静止 none | `0.00000 m` | `0.00006 m` | `0.00000 deg` | `0` | 静止状态正常 |
| 前进 forward | `+0.34325 m` | `+0.00183 m` | `-0.01000 deg` | `0` | 直行 y 漂约 1.8 mm，正常 |
| 后退 backward | `-0.11380 m` | `-0.00628 m` | `-0.18000 deg` | `0` | y 变化由初始 yaw 偏角导致 |
| 原地旋转 rotate_ccw | `-0.00136 m` | `+0.00101 m` | `+10.83000 deg` | `0` | 原地旋转 position 变化约 1 mm，正常 |

其中最关键的是前进测试：

```text
position.x: 0.55749 -> 0.90074, delta = +0.34325 m
position.y: 0.00268 -> 0.00451, delta = +0.00183 m
velocity.vbx: 0.14~0.16 m/s
velocity.vby: 基本为 0，偶尔 -0.01 m/s
attitude.yaw: -0.07~0.04 deg
slip_flag: 0
```

这说明直行时 EP SDK 原始横向漂移非常小，不是主要问题。

## 3. 为什么后退时 x/y 都变是正常现象

后退测试中，`position.x` 和 `position.y` 都变化了：

```text
position.x: -0.62305 -> -0.73685, delta = -0.11380 m
position.y: -0.06695 -> -0.07323, delta = -0.00628 m
yaw: 约 3.1~3.4 deg
```

这不是异常漂移。原因是车头本来就相对 odom 的 x 轴偏了约 `3.3 deg`。

如果机器人沿自身前后方向运动 `0.114 m`，在世界坐标中产生的横向分量约为：

```text
0.114 * sin(3.3 deg) = 0.0065 m
```

实测：

```text
position.y delta = 0.00628 m
```

二者基本一致。

因此：

```text
只要车头没有完全对齐 odom x 轴，机器人前进/后退时 x 和 y 同时变化是正常几何结果。
```

这也说明 EP SDK 的 `sub_position` 与 `sub_attitude` 在这次测试里是自洽的。

## 4. 现有问题的最可能原因

### 4.1 没有做启动原点归零

错误做法：

```python
odom.pose.pose.position.x = ep_x
odom.pose.pose.position.y = ep_y
```

这样 ROS 看到的 odom 初始位置不是 `(0, 0)`，而是 SDK 当前累计位置。

正确做法：

```python
x0 = 第一次收到的 ep_x
y0 = 第一次收到的 ep_y

dx = ep_x - x0
dy = ep_y - y0
```

ROS `/odom` 应该从当前启动位置开始算：

```text
启动时 odom.pose.position 接近 0, 0
```

### 4.2 没有扣除初始 yaw

如果启动时车头已经偏了 `3 deg`，但 ROS 没有把这个偏角作为初始朝向处理，那么之后前进/后退时就会看到 x/y 同时变化。

正确做法：

```python
yaw0 = 第一次收到的 ep_yaw
yaw = ep_yaw - yaw0
```

### 4.3 没有把 EP 坐标旋转到 ROS odom 初始坐标系

只做平移归零还不够。如果启动时车头有偏角，还要做坐标旋转。

基本思想：

```text
EP SDK 返回的是 EP 自己的平面坐标变化。
ROS odom 应该以启动瞬间为原点，并以启动瞬间车头方向作为初始 x 轴。
```

### 4.4 yaw 角度单位可能错误

EP SDK 的 `sub_attitude` 返回的是角度 `degree`。

ROS 的四元数函数通常需要弧度 `radian`。

错误做法：

```python
q = tf.transformations.quaternion_from_euler(0, 0, yaw_deg)
```

正确做法：

```python
yaw_rad = math.radians(yaw_deg)
q = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
```

### 4.5 cmd_vel 和 odom 的坐标符号不一致

很多 RoboMaster ROS 实现中，控制方向会这样转：

```python
drive_speed(
    x=cmd.linear.x,
    y=-cmd.linear.y,
    z=-math.degrees(cmd.angular.z)
)
```

那么 odom 回传也必须使用一致的 EP -> ROS 坐标转换。

如果控制端反号，回传端不反号，就会出现：

```text
ROS 发的方向是对的，但 ROS 看到的 odom 方向是错的。
```

### 4.6 odom.twist 速度来源可能不正确

EP SDK `sub_velocity` 返回：

```text
vgx, vgy, vgz, vbx, vby, vbz
```

建议第一版导航使用车体系速度：

```text
odom.twist.twist.linear.x = vbx
odom.twist.twist.linear.y = 0.0
```

先不要启用横移：

```text
linear.y = 0
```

### 4.7 重复发布 odom -> base_link TF

如果 EP driver 和 EKF 同时发布：

```text
odom -> base_link
```

TF 会冲突。

必须二选一：

```text
方案 A：EP odom driver 发布 odom -> base_link，EKF 不发布。
方案 B：EKF 发布 odom -> base_link，EP odom driver 不发布。
```

第一版不接 EKF 时，建议由 EP odom driver 发布。

## 5. 推荐的最优解决方案

当前测试数据说明 EP `sub_position` 可以保留为主位置来源。

推荐数据链路：

```text
sub_position  -> odom.pose.position
sub_attitude  -> odom.pose.orientation
sub_velocity  -> odom.twist.linear
sub_imu       -> /ep/imu，同时提供 odom.twist.angular
```

但必须加入：

```text
1. position 原点归零
2. yaw 初始归零
3. 坐标旋转
4. degree -> radian
5. EP -> ROS 坐标符号统一
6. odom.twist 填真实速度
7. 只保留一个 odom -> base_link TF 发布者
```

## 6. Odom 修正算法

第一次收到 SDK 数据时记录：

```python
self.x0 = ep_x
self.y0 = ep_y
self.yaw0 = math.radians(ep_yaw_deg)
```

后续每帧：

```python
ep_yaw = math.radians(ep_yaw_deg)

dx_ep = ep_x - self.x0
dy_ep = ep_y - self.y0
```

根据实测方向做 EP -> ROS y 轴转换。常见写法：

```python
dx = dx_ep
dy = -dy_ep
```

扣除初始 yaw：

```python
yaw = ep_yaw - self.yaw0
```

把平移量旋转到 ROS odom 初始坐标系：

```python
c = math.cos(-self.yaw0)
s = math.sin(-self.yaw0)

odom_x = c * dx - s * dy
odom_y = s * dx + c * dy
```

生成四元数：

```python
q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
```

速度：

```python
odom.twist.twist.linear.x = vbx
odom.twist.twist.linear.y = 0.0
odom.twist.twist.angular.z = gyro_z
```

注意：`gyro_z` 是否需要反号，必须按实车验证。

## 7. yaw 和 gyro_z 符号验证

不要仅凭别人仓库的代码决定符号。按实际物理动作判断。

测试方法：

```text
让车原地逆时针旋转。
```

如果 SDK 中：

```text
yaw_deg 增加
gyro_z 为正
```

ROS 中就可以先用：

```python
yaw = math.radians(ep_yaw_deg - yaw0_deg)
wz = gyro_z
```

如果物理逆时针旋转时：

```text
yaw_deg 减小
gyro_z 为负
```

则需要反号：

```python
yaw = -math.radians(ep_yaw_deg - yaw0_deg)
wz = -gyro_z
```

本次 `rotate_ccw` 测试中：

```text
yaw_deg: 87.53 -> 98.36
gyro_z: 约 +0.31 ~ +0.38
```

说明当前 SDK 数据里逆时针旋转时 yaw 增加、gyro_z 为正。

因此当前这台车优先使用：

```python
yaw = math.radians(ep_yaw_deg - yaw0_deg)
wz = gyro_z
```

## 8. 第一版导航配置建议

先把 EP 当差速车调通。

`cmd_vel` 只使用：

```text
linear.x
angular.z
```

禁用横向速度：

```yaml
max_vel_y: 0.0
min_vel_y: 0.0
holonomic_robot: false
```

`odom.twist` 第一版：

```text
linear.x = vbx
linear.y = 0.0
angular.z = gyro_z
```

等定位和导航稳定后，再考虑开启麦克纳姆横移。

## 9. TF 发布策略

不使用 EKF 时：

```text
EP odom driver 发布 odom -> base_link
```

使用 EKF 时：

```text
EP odom driver 只发布 /ep/odom，不发布 TF
robot_localization 发布 odom -> base_link
```

检查是否重复发布：

```bash
rosrun tf view_frames
rosrun tf tf_echo odom base_link
```

如果 TF 抖动或跳变，优先排查是否有两个节点同时发布同一条 TF。

## 10. 修正后的验收标准

### 10.1 启动归零

启动 ROS odom 后：

```text
/ep/odom.pose.pose.position.x 接近 0
/ep/odom.pose.pose.position.y 接近 0
yaw 接近 0
```

### 10.2 静止测试

静止 60 秒：

```text
x/y 漂移 < 0.01 m
yaw 漂移 < 0.5 deg
```

### 10.3 直行测试

沿胶带前进 1 m：

```text
x 接近 1.0 m
y 接近 0
```

如果启动时车头没有严格对齐胶带，y 有少量变化是正常的。

### 10.4 初始偏角测试

故意让车头偏 `3 deg` 前进：

```text
x/y 都变化是正常现象。
y / x 应接近 tan(3 deg)。
```

### 10.5 原地旋转测试

原地逆时针旋转 90 deg：

```text
yaw 增加约 +1.57 rad
x/y 漂移 < 0.02 m
```

## 11. 是否需要外置 IMU

基于当前测试，暂时不是必须。

理由：

```text
静止 yaw 稳定。
原地旋转 yaw 和 gyro_z 同向且数值合理。
直行时 yaw 变化很小。
```

如果后续出现以下情况，再考虑 HI04：

```text
1. 电机运行时 yaw 明显跳变。
2. 静止 yaw 长时间漂移明显。
3. 原地旋转 gyro_z 噪声大或方向不稳定。
4. AMCL / EKF 对 yaw 观测明显不信任。
```

加 HI04 后的推荐结构是：

```text
HI04             -> yaw / yaw_rate
EP sub_velocity  -> vbx
EP sub_position  -> debug 或低权重 position
EKF              -> /odometry/filtered + odom -> base_link
```

## 12. 最终处理优先级

按这个顺序做：

1. 修改 ROS odom 节点，加入 `x0/y0/yaw0` 归零。
2. 修正 `yaw_deg -> yaw_rad`。
3. 按本次实测结果确定 yaw/gyro_z 符号。
4. 用 `sub_velocity.vbx` 填 `odom.twist.linear.x`。
5. 第一版强制 `odom.twist.linear.y = 0`。
6. 只保留一个 `odom -> base_link` TF 发布者。
7. 重新做静止、直行、后退、旋转验收。
8. 验收通过后再接 gmapping、AMCL、move_base。

一句话结论：

```text
当前问题不是 EP SDK 原始数据坏了，而是 ROS odom 应该做启动归零、初始 yaw 扣除、坐标旋转、单位转换和符号统一。
```


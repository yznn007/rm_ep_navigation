#!/usr/bin/env python3
"""
RoboMaster EP 底盘驱动 - ROS1 版本
基于 ROS2 robomaster_ros 驱动移植
坐标映射与 ROS2 完全一致
"""

import math
import threading
import time
from typing import Optional, Tuple, List

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler

try:
    from robomaster import robot as rm_robot
    from robomaster import protocol
    from robomaster import conn as rm_conn
    SDK_AVAILABLE = True
except ImportError:
    rospy.logwarn("robomaster SDK 未安装，请执行: pip3 install robomaster")
    SDK_AVAILABLE = False


# 常量定义
RADIUS = 0.05  # 轮子半径 (m)
AXIS = 0.2     # 轴距 (m)
RPM2SPEED = 2 * math.pi * RADIUS / 60
G = 9.81       # 重力加速度


def rpm_from_linear_speed(value: float) -> int:
    """线速度 (m/s) -> RPM"""
    return round(value / RPM2SPEED)


def linear_speed_from_rpm(value: float) -> float:
    """RPM -> 线速度 (m/s)"""
    return RPM2SPEED * value


def wheel_speeds_from_twist(vx: float, vy: float, vtheta: float,
                            axis_length: float = AXIS) -> Tuple[float, float, float, float]:
    """将 twist 转换为麦轮速度"""
    front_left  = vx - vy - axis_length * vtheta
    front_right = vx + vy + axis_length * vtheta
    rear_left   = vx + vy - axis_length * vtheta
    rear_right  = vx - vy + axis_length * vtheta
    return (front_left, front_right, rear_left, rear_right)


WHEEL_FRAMES = ['front_right_wheel_joint', 'front_left_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint']


class RmEpChassisDriver:
    """RoboMaster EP 底盘驱动 - ROS1 版本"""

    def __init__(self):
        rospy.init_node("rm_ep_chassis_driver", anonymous=False)

        # 加载参数
        self._load_params()

        # 状态变量
        self._lock = threading.Lock()
        self._position = None
        self._attitude = None
        self._velocity = None
        self._imu_data = None
        self._esc_data = None
        self._last_cmd_time = rospy.Time.now()
        self._yaw = None
        self._position_xy = None

        # odom 归零变量
        self._x0 = None
        self._y0 = None
        self._yaw0 = None

        # ROS 发布者
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self._tf_broadcaster = TransformBroadcaster()

        # ROS 订阅者
        if self.enable_cmd_vel:
            self.cmd_vel_sub = rospy.Subscriber(
                "/cmd_vel", Twist, self._cmd_vel_callback, queue_size=1
            )

        # 消息初始化
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_id
        self.odom_msg.child_frame_id = self.base_frame_id

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.imu_frame_id

        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = self.odom_frame_id
        self.transform_msg.child_frame_id = self.base_frame_id

        # 设置协方差
        self._setup_covariances()

        # 连接 EP
        self._connect_ep()
        self._start_data_streams()

        # 超时检查定时器
        self._cmd_vel_timer = rospy.Timer(
            rospy.Duration(0.1), self._check_cmd_vel_timeout
        )

        rospy.loginfo("rm_ep_chassis_driver 初始化完成")

    def _load_params(self):
        """加载 ROS 参数"""
        self.ep_sn = rospy.get_param("~ep_sn", "")
        self.ep_conn_type = rospy.get_param("~ep_conn_type", "rndis")
        self.ep_ip = rospy.get_param("~ep_ip", "")
        self.enable_cmd_vel = rospy.get_param("~enable_cmd_vel", True)
        self.odom_rate = rospy.get_param("~odom_rate", 20)
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.imu_frame_id = rospy.get_param("~imu_frame_id", "imu_link")
        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 0.5)
        self.twist_to_wheel_speeds = rospy.get_param("~twist_to_wheel_speeds", False)
        self.force_level = rospy.get_param("~force_level", False)
        self.imu_has_orientation = rospy.get_param("~imu_has_orientation", True)

        # 协方差参数
        self.linear_velocity_error = rospy.get_param("~linear_velocity_error", 0.005)
        self.angular_velocity_error_xy = rospy.get_param("~angular_velocity_error_xy", 0.01)
        self.angular_velocity_error_z = rospy.get_param("~angular_velocity_error_z", 0.03)
        self.linear_acceleration_error = rospy.get_param("~linear_acceleration_error", 0.1)

    def _setup_covariances(self):
        """设置协方差矩阵"""
        # 速度协方差
        lv = self.linear_velocity_error ** 2
        av_xy = self.angular_velocity_error_xy ** 2
        av_z = self.angular_velocity_error_z ** 2
        la = self.linear_acceleration_error ** 2

        # Twist 协方差
        vs = self.odom_msg.twist.covariance
        vs[0] = vs[7] = lv      # linear.x, linear.y
        vs[21] = vs[28] = av_xy  # angular.x, angular.y
        vs[35] = av_z            # angular.z

        # Pose 协方差
        ps = self.odom_msg.pose.covariance
        ps[0] = ps[7] = lv
        ps[21] = ps[28] = av_xy
        ps[35] = av_z

        # IMU 协方差
        ivs = self.imu_msg.angular_velocity_covariance
        ivs[0] = ivs[4] = av_xy
        ivs[8] = av_z

        ias = self.imu_msg.linear_acceleration_covariance
        ias[0] = ias[4] = ias[8] = la

        ioc = self.imu_msg.orientation_covariance
        ioc[0] = ioc[4] = ioc[8] = 0.01

    def _connect_ep(self):
        """连接 RoboMaster EP"""
        if not SDK_AVAILABLE:
            rospy.logerr("RoboMaster SDK 不可用，无法连接 EP")
            raise RuntimeError("RoboMaster SDK not available")

        self.ep_robot = rm_robot.Robot()

        # 将 conn_type 字符串映射到 SDK 常量 (SDK 使用 is 比较，必须用常量)
        conn_type_map = {
            'ap': rm_conn.CONNECTION_WIFI_AP,
            'sta': rm_conn.CONNECTION_WIFI_STA,
            'rndis': rm_conn.CONNECTION_USB_RNDIS,
        }
        conn_type = conn_type_map.get(self.ep_conn_type, self.ep_conn_type)
        sn = self.ep_sn if self.ep_sn else None

        rospy.loginfo("正在连接 RoboMaster EP (conn_type=%s, sn=%s)...", conn_type, sn)
        try:
            self.ep_robot.initialize(conn_type=conn_type, sn=sn)
            version = self._safe_call(self.ep_robot.get_version)
            rospy.loginfo("RoboMaster EP 连接成功, 固件版本: %s", version)
        except Exception as e:
            rospy.logerr("连接 RoboMaster EP 失败: %s", e)
            raise

        try:
            battery = self._safe_call(self.ep_robot.battery.get_battery)
            rospy.loginfo("电池电量: %s%%", battery)
        except Exception:
            rospy.logwarn("无法获取电池信息")

    def _start_data_streams(self):
        """启动数据流订阅"""
        chassis = self.ep_robot.chassis
        freq = max(1, min(50, self.odom_rate))

        # 订阅位置、姿态、速度、IMU、电调数据
        chassis.sub_position(cs=1, freq=freq, callback=self._position_callback)
        chassis.sub_attitude(freq=freq, callback=self._attitude_callback)
        chassis.sub_velocity(freq=freq, callback=self._velocity_callback)
        chassis.sub_imu(freq=freq, callback=self._imu_callback)
        chassis.sub_esc(freq=freq, callback=self._esc_callback)

        # 发布定时器
        rospy.Timer(
            rospy.Duration(1.0 / max(1, freq)),
            self._publish_timer_callback
        )

        rospy.loginfo("数据流已启动 (freq=%d Hz)", freq)

    def _position_callback(self, position_info):
        """位置回调 - SDK 返回 (x, y, z)"""
        with self._lock:
            try:
                self._position = (
                    float(position_info[0]),
                    float(position_info[1]),
                    float(position_info[2]),
                )
            except (TypeError, IndexError):
                try:
                    self._position = (
                        float(position_info.x),
                        float(position_info.y),
                        float(position_info.z),
                    )
                except AttributeError:
                    pass

    def _attitude_callback(self, attitude_info):
        """姿态回调 - SDK 返回 (yaw, pitch, roll)"""
        with self._lock:
            try:
                self._attitude = (
                    float(attitude_info[0]),
                    float(attitude_info[1]),
                    float(attitude_info[2]),
                )
            except (TypeError, IndexError):
                try:
                    self._attitude = (
                        float(attitude_info.yaw),
                        float(attitude_info.pitch),
                        float(attitude_info.roll),
                    )
                except AttributeError:
                    pass

    def _velocity_callback(self, velocity_info):
        """速度回调 - SDK 返回 (vgx, vgy, vgz, vbx, vby, vbz)"""
        with self._lock:
            try:
                self._velocity = (
                    float(velocity_info[0]),  # vgx
                    float(velocity_info[1]),  # vgy
                    float(velocity_info[2]),  # vgz
                    float(velocity_info[3]),  # vbx
                    float(velocity_info[4]),  # vby
                    float(velocity_info[5]),  # vbz
                )
            except (TypeError, IndexError):
                try:
                    self._velocity = (
                        float(velocity_info.vgx),
                        float(velocity_info.vgy),
                        float(velocity_info.vgz),
                        float(velocity_info.vbx),
                        float(velocity_info.vby),
                        float(velocity_info.vbz),
                    )
                except AttributeError:
                    pass

    def _imu_callback(self, imu_info):
        """IMU 回调 - SDK 返回 (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)"""
        with self._lock:
            try:
                self._imu_data = (
                    float(imu_info[0]),  # acc_x
                    float(imu_info[1]),  # acc_y
                    float(imu_info[2]),  # acc_z
                    float(imu_info[3]),  # gyro_x
                    float(imu_info[4]),  # gyro_y
                    float(imu_info[5]),  # gyro_z
                )
            except (TypeError, IndexError):
                try:
                    self._imu_data = (
                        float(imu_info.acc_x),
                        float(imu_info.acc_y),
                        float(imu_info.acc_z),
                        float(imu_info.gyro_x),
                        float(imu_info.gyro_y),
                        float(imu_info.gyro_z),
                    )
                except AttributeError:
                    pass

    def _esc_callback(self, esc_info):
        """电调回调 - SDK 返回 (speed[4], angle[4], timestamp[4], state[4])"""
        with self._lock:
            try:
                self._esc_data = (
                    [int(esc_info[0][i]) for i in range(4)],  # speeds
                    [int(esc_info[1][i]) for i in range(4)],  # angles
                    [int(esc_info[2][i]) for i in range(4)],  # timestamps
                    [int(esc_info[3][i]) for i in range(4)],  # states
                )
            except (TypeError, IndexError):
                pass

    def _publish_timer_callback(self, event):
        """定时发布 odom 和 IMU"""
        now = rospy.Time.now()

        with self._lock:
            pos = self._position
            att = self._attitude
            vel = self._velocity
            imu = self._imu_data

        if pos is not None and att is not None:
            self._update_odom(pos, att, vel)
            self._publish_state_estimation(now)

    def _update_odom(self, pos, att, vel):
        """更新里程计数据 - 归零+旋转+车体系速度"""
        px, py, pz = pos
        yaw_deg, pitch_deg, roll_deg = att

        # 首次记录原点
        if self._x0 is None:
            self._x0 = px
            self._y0 = py
            self._yaw0 = math.radians(yaw_deg)
            rospy.loginfo("odom 原点已记录: x0=%.3f, y0=%.3f, yaw0=%.3f deg",
                          self._x0, self._y0, yaw_deg)

        # 归零
        dx_ep = px - self._x0
        dy_ep = py - self._y0
        yaw_ep = math.radians(yaw_deg) - self._yaw0

        # EP->ROS: y 取反
        dx = dx_ep
        dy = -dy_ep

        # 旋转到启动坐标系
        c = math.cos(-self._yaw0)
        s = math.sin(-self._yaw0)
        odom_x = c * dx - s * dy
        odom_y = s * dx + c * dy

        # 设置位置
        position = self.odom_msg.pose.pose.position
        position.x = odom_x
        position.y = odom_y
        self._position_xy = (odom_x, odom_y)

        # yaw 扣除初始偏角
        yaw = -yaw_ep
        self._yaw = yaw

        # 姿态四元数
        if self.force_level:
            pitch = 0.0
            roll = 0.0
        else:
            pitch = -math.radians(pitch_deg)
            roll = math.radians(roll_deg)

        q = quaternion_from_euler(roll, pitch, yaw)
        orientation = self.odom_msg.pose.pose.orientation
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]

        # 速度: 车体系 vbx, -vby
        if vel is not None:
            velocity = self.odom_msg.twist.twist.linear
            velocity.x = vel[3]   # vbx
            velocity.y = -vel[4]  # -vby

        # 角速度使用 IMU 的 gyro_z (在 _publish_state_estimation 中设置)

    def _publish_state_estimation(self, stamp):
        """发布状态估计"""
        self.odom_msg.header.stamp = stamp

        if self.imu_has_orientation:
            self.imu_msg.orientation = self.odom_msg.pose.pose.orientation

        # IMU 数据映射
        if self._imu_data is not None:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self._imu_data
            acceleration = self.imu_msg.linear_acceleration
            # 加速度: ax 直接, ay 取反, az 取反
            acceleration.x = acc_x * G
            acceleration.y = -acc_y * G
            acceleration.z = -acc_z * G

            angular_speed = self.imu_msg.angular_velocity
            # 角速度: gx 直接, gy 取反, gz 取反
            angular_speed.x = math.radians(gyro_x)
            angular_speed.y = -math.radians(gyro_y)
            angular_speed.z = -math.radians(gyro_z)

            # odom 的角速度也使用 IMU 的 gyro_z
            self.odom_msg.twist.twist.angular.z = -math.radians(gyro_z)

        # 发布 odom 和 IMU (带异常处理)
        try:
            self.odom_pub.publish(self.odom_msg)
            self.imu_msg.header.stamp = stamp
            self.imu_pub.publish(self.imu_msg)
        except Exception:
            pass  # 话题关闭时忽略错误

        # TF 发布已禁用：由 EKF (robot_localization) 统一发布 odom->base_link
        # 避免与 EKF 的 TF 冲突
        # position = self.odom_msg.pose.pose.position
        # translation = self.transform_msg.transform.translation
        # translation.x = position.x
        # translation.y = position.y
        # translation.z = position.z
        # self.transform_msg.transform.rotation = self.odom_msg.pose.pose.orientation
        # self.transform_msg.header.stamp = stamp
        # self._tf_broadcaster.sendTransform(self.transform_msg)

    def _cmd_vel_callback(self, msg):
        """速度指令回调 - 坐标映射与 ROS2 一致"""
        if not hasattr(self, "ep_robot") or self.ep_robot is None:
            return

        self._last_cmd_time = rospy.Time.now()

        if self.twist_to_wheel_speeds:
            # 麦轮速度控制模式
            front_left, front_right, rear_left, rear_right = wheel_speeds_from_twist(
                msg.linear.x, msg.linear.y, msg.angular.z)
            try:
                self.ep_robot.chassis.drive_wheels(
                    w1=rpm_from_linear_speed(front_right),
                    w2=rpm_from_linear_speed(front_left),
                    w3=rpm_from_linear_speed(rear_left),
                    w4=rpm_from_linear_speed(rear_right),
                    timeout=self.cmd_vel_timeout)
            except Exception as e:
                rospy.logwarn_throttle(5.0, "发送麦轮速度指令失败: %s", e)
        else:
            # 底盘速度控制模式
            # 坐标映射: x 直接, y 取反, yaw 取反
            vx = msg.linear.x
            vy = -msg.linear.y
            vz_rad = -msg.angular.z
            vz_deg = math.degrees(vz_rad)

            # 限幅
            max_v = 3.5
            vx = max(-max_v, min(max_v, vx))
            vy = max(-max_v, min(max_v, vy))
            vz_deg = max(-600, min(600, vz_deg))

            try:
                self.ep_robot.chassis.drive_speed(
                    x=vx, y=vy, z=vz_deg,
                    timeout=self.cmd_vel_timeout
                )
            except Exception as e:
                rospy.logwarn_throttle(5.0, "发送速度指令失败: %s", e)

    def _check_cmd_vel_timeout(self, event):
        """检查速度指令超时"""
        if not self.enable_cmd_vel or not hasattr(self, "ep_robot"):
            return

        elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
        if elapsed > self.cmd_vel_timeout:
            try:
                self.ep_robot.chassis.drive_speed(x=0, y=0, z=0, timeout=0)
            except Exception:
                pass

    def _safe_call(self, func, *args, **kwargs):
        """安全调用函数"""
        try:
            return func(*args, **kwargs)
        except Exception as e:
            rospy.logwarn("调用 %r 失败: %s", func.__name__, e)
            return None

    def shutdown(self):
        """关闭驱动"""
        rospy.loginfo("rm_ep_chassis_driver 正在关闭...")
        if hasattr(self, "ep_robot") and self.ep_robot is not None:
            try:
                self.ep_robot.chassis.drive_speed(x=0, y=0, z=0, timeout=0)
                self.ep_robot.close()
            except Exception:
                pass
        rospy.loginfo("rm_ep_chassis_driver 已关闭")


if __name__ == "__main__":
    driver = None
    try:
        driver = RmEpChassisDriver()
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("驱动节点异常退出: %s", e)
        if driver is not None:
            driver.shutdown()

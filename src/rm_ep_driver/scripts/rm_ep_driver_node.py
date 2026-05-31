#!/usr/bin/env python3
"""RoboMaster EP ROS 驱动节点"""

import math
import threading
import time

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError

try:
    from robomaster import robot as rm_robot
    SDK_AVAILABLE = True
except ImportError:
    rospy.logwarn("robomaster SDK 未安装，请执行: pip3 install robomaster")
    SDK_AVAILABLE = False


def _quat_from_axis_angle(axis, angle_deg):
    """轴角转四元数，返回 (x, y, z, w)"""
    half = math.radians(angle_deg) / 2.0
    s = math.sin(half)
    c = math.cos(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, c)


def _quat_multiply(q1, q2):
    """四元数乘法 q1 * q2"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_inverse(q):
    """四元数逆（单位四元数 = 共轭）"""
    return (-q[0], -q[1], -q[2], q[3])


class RmEpDriver:
    """RoboMaster EP ROS 驱动"""

    def __init__(self):
        rospy.init_node("rm_ep_driver", anonymous=False)

        self._load_params()

        self._lock = threading.Lock()
        self._position = None
        self._attitude = None
        self._velocity = None
        self._imu_data = None
        self._last_cmd_time = rospy.Time.now()
        self._last_attitude_q = None
        self._init_orientation = None

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self._tf_broadcaster = TransformBroadcaster()

        if self.enable_cmd_vel:
            self.cmd_vel_sub = rospy.Subscriber(
                "/cmd_vel_rm_ep", Twist, self._cmd_vel_callback, queue_size=1
            )

        self._cv_bridge = CvBridge()
        self._img_pub = None
        self._joint_pub = None
        if self._enable_camera:
            self._img_pub = rospy.Publisher(
                "/camera/image_raw", Image, queue_size=3
            )
        if self._enable_gimbal:
            self._joint_pub = rospy.Publisher(
                "/joint_states", JointState, queue_size=3
            )

        self._connect_ep()
        self._start_data_streams()

        self._cmd_vel_timer = rospy.Timer(
            rospy.Duration(0.1), self._check_cmd_vel_timeout
        )

        rospy.loginfo("rm_ep_driver 初始化完成")

    def _load_params(self):
        self.ep_sn = rospy.get_param("~ep_sn", "")
        self.ep_conn_type = rospy.get_param("~ep_conn_type", "ap")
        self.ep_ip = rospy.get_param("~ep_ip", "")
        self.enable_cmd_vel = rospy.get_param("~enable_cmd_vel", True)
        self.odom_rate = rospy.get_param("~odom_rate", 20)
        self.imu_rate = rospy.get_param("~imu_rate", 20)
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.imu_frame_id = rospy.get_param("~imu_frame_id", "imu_link")
        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 0.5)

        self._enable_camera = rospy.get_param("~enable_camera", True)
        self._camera_frame_id = rospy.get_param("~camera_frame_id", "camera_link_optical_frame")
        self._enable_gimbal = rospy.get_param("~enable_gimbal", True)
        self._gimbal_rate = rospy.get_param("~gimbal_rate", 50)
        self._init_attitude_calibration = rospy.get_param("~init_attitude_calibration", True)
        self._imu_gravity_constant = rospy.get_param("~imu_gravity_constant", 9.86)
        self._imu_flip_x = rospy.get_param("~imu_flip_x", False)
        self._imu_flip_y = rospy.get_param("~imu_flip_y", False)
        self._yaw_offset_deg = rospy.get_param("~yaw_offset_deg", 0.0)

    def _connect_ep(self):
        if not SDK_AVAILABLE:
            rospy.logerr("RoboMaster SDK 不可用，无法连接 EP")
            raise RuntimeError("RoboMaster SDK not available")

        self.ep_robot = rm_robot.Robot()

        conn_type = self.ep_conn_type
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
        chassis = self.ep_robot.chassis
        freq = max(5, min(20, self.odom_rate))

        chassis.sub_position(
            freq=freq, callback=self._position_callback
        )
        chassis.sub_attitude(
            freq=freq, callback=self._attitude_callback
        )

        try:
            chassis.sub_velocity(
                freq=freq, callback=self._velocity_callback
            )
        except Exception:
            rospy.logwarn("无法订阅速度数据流，将使用位置差分计算速度")
            pass

        try:
            chassis.sub_imu(
                freq=freq, callback=self._imu_callback
            )
        except Exception:
            rospy.logwarn("无法订阅 IMU 数据流")
            pass

        rospy.Timer(
            rospy.Duration(1.0 / max(1, self.odom_rate)),
            self._publish_timer_callback
        )

        if self._enable_camera:
            try:
                self.ep_robot.camera.start_video_stream(display=False)
                self._cam_thread = threading.Thread(target=self._cam_loop)
                self._cam_thread.daemon = True
                self._cam_thread.start()
                rospy.loginfo("相机流已启动, 发布到 /camera/image_raw")
            except Exception as e:
                rospy.logwarn("无法启动相机流: %s", e)

        if self._enable_gimbal:
            try:
                self.ep_robot.gimbal.sub_angle(
                    freq=self._gimbal_rate, callback=self._gimbal_angle_callback
                )
                rospy.loginfo("云台角度订阅已启动 (freq=%d)", self._gimbal_rate)
            except Exception as e:
                rospy.logwarn("无法订阅云台角度: %s", e)

    def _get_value(self, info, attr, default=0.0):
        """安全获取属性值，兼容 (x, y, z) 元组"""
        try:
            return float(info[attr])
        except (TypeError, KeyError, IndexError):
            try:
                return float(getattr(info, attr, default))
            except (TypeError, ValueError):
                return default

    def _position_callback(self, position_info):
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

            if self._attitude is not None:
                yaw_d, pitch_d, roll_d = self._attitude
                qz = _quat_from_axis_angle((0, 0, 1), -(yaw_d + self._yaw_offset_deg))
                qy = _quat_from_axis_angle((0, 1, 0), pitch_d)
                qx = _quat_from_axis_angle((1, 0, 0), roll_d)
                self._last_attitude_q = _quat_multiply(_quat_multiply(qz, qy), qx)

                if self._init_attitude_calibration and self._init_orientation is None:
                    self._init_orientation = self._last_attitude_q
                    rospy.loginfo("初始姿态已记录 (yaw=%.1f, pitch=%.1f, roll=%.1f)",
                                  yaw_d, pitch_d, roll_d)

    def _velocity_callback(self, velocity_info):
        with self._lock:
            try:
                self._velocity = (
                    float(velocity_info[0]),
                    float(velocity_info[1]),
                    float(velocity_info[2]),
                )
            except (TypeError, IndexError):
                try:
                    self._velocity = (
                        float(velocity_info.vgx),
                        float(velocity_info.vgy),
                        float(velocity_info.vgz),
                    )
                except AttributeError:
                    pass

    def _imu_callback(self, imu_info):
        with self._lock:
            try:
                acc_x = float(imu_info[0])
                acc_y = float(imu_info[1])
                acc_z = float(imu_info[2])
                gyro_x = float(imu_info[3])
                gyro_y = float(imu_info[4])
                gyro_z = float(imu_info[5])
            except (TypeError, IndexError):
                try:
                    acc_x = float(imu_info.acc_x)
                    acc_y = float(imu_info.acc_y)
                    acc_z = float(imu_info.acc_z)
                    gyro_x = float(imu_info.gyro_x)
                    gyro_y = float(imu_info.gyro_y)
                    gyro_z = float(imu_info.gyro_z)
                except AttributeError:
                    pass
                else:
                    g = self._imu_gravity_constant
                    if g > 0:
                        self._imu_data = (
                            acc_x * g, acc_y * g, acc_z * g,
                            gyro_x, gyro_y, gyro_z,
                        )
                    else:
                        self._imu_data = (
                            acc_x, acc_y, acc_z,
                            gyro_x, gyro_y, gyro_z,
                        )
                    return
            else:
                g = self._imu_gravity_constant
                if g > 0:
                    self._imu_data = (
                        acc_x * g, acc_y * g, acc_z * g,
                        gyro_x, gyro_y, gyro_z,
                    )
                else:
                    self._imu_data = (
                        acc_x, acc_y, acc_z,
                        gyro_x, gyro_y, gyro_z,
                    )

    def _publish_timer_callback(self, event):
        now = rospy.Time.now()

        with self._lock:
            pos = self._position
            att = self._attitude
            vel = self._velocity
            imu = self._imu_data

        if pos is not None and att is not None:
            self._publish_odometry(pos, att, vel, now)

        if imu is not None or att is not None:
            self._publish_imu(imu, att, now)

    def _quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    @staticmethod
    def _quaternion_to_yaw(q):
        x, y, z, w = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _publish_odometry(self, pos, att, vel, now):
        px, py, pz = pos
        yaw_deg, pitch_deg, roll_deg = att
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = py
        odom.pose.pose.position.y = px
        odom.pose.pose.position.z = 0.0
        if self._init_attitude_calibration and self._last_attitude_q is not None and self._init_orientation is not None:
            q = _quat_multiply(self._last_attitude_q, _quat_inverse(self._init_orientation))
            odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            calibrated_yaw = self._quaternion_to_yaw(q)
        else:
            odom.pose.pose.orientation = self._quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg + self._yaw_offset_deg))
            calibrated_yaw = math.radians(yaw_deg + self._yaw_offset_deg)

        odom.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]

        if vel is not None:
            vx, vy, vz = vel
            vx_global = vy
            vy_global = vx
            cos_yaw = math.cos(calibrated_yaw)
            sin_yaw = math.sin(calibrated_yaw)
            odom.twist.twist.linear.x = vx_global
            odom.twist.twist.linear.y = vy_global
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = -math.radians(vz)
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0

        odom.twist.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]

        self.odom_pub.publish(odom)

        # TF 广播已禁用：EKF 启用时由 ekf_localization_node 发布 odom→base_link
        # t = TransformStamped()
        # t.header.stamp = now
        # t.header.frame_id = self.odom_frame_id
        # t.child_frame_id = self.base_frame_id
        # t.transform.translation.x = odom.pose.pose.position.y
        # t.transform.translation.y = odom.pose.pose.position.x
        # t.transform.translation.z = odom.pose.pose.position.z
        # t.transform.rotation = odom.pose.pose.orientation
        # self._tf_broadcaster.sendTransform(t)

    def _publish_imu(self, imu, att, now):
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.imu_frame_id

        if att is not None:
            if self._init_attitude_calibration and self._last_attitude_q is not None and self._init_orientation is not None:
                q = _quat_multiply(self._last_attitude_q, _quat_inverse(self._init_orientation))
                imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            else:
                yaw_deg, pitch_deg, roll_deg = att
                yaw = math.radians(yaw_deg)
                pitch = math.radians(pitch_deg)
                roll = math.radians(roll_deg)
                imu_msg.orientation = self._quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        if imu is not None:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu
            if self._imu_flip_x:
                acc_x = -acc_x
                gyro_x = -gyro_x
            if self._imu_flip_y:
                acc_y = -acc_y
                gyro_y = -gyro_y
            imu_msg.angular_velocity.x = math.radians(gyro_x)
            imu_msg.angular_velocity.y = math.radians(gyro_y)
            imu_msg.angular_velocity.z = -math.radians(gyro_z)
            imu_msg.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]

            imu_msg.linear_acceleration.x = acc_x
            imu_msg.linear_acceleration.y = acc_y
            imu_msg.linear_acceleration.z = acc_z
            imu_msg.linear_acceleration_covariance = [0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05]
        elif att is not None:
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

        self.imu_pub.publish(imu_msg)

    def _cam_loop(self):
        while not rospy.is_shutdown():
            try:
                img = self.ep_robot.camera.read_cv2_image(timeout=0.1, strategy='newest')
                if img is None:
                    continue
                msg = self._cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self._camera_frame_id
                self._img_pub.publish(msg)
            except CvBridgeError as e:
                rospy.logwarn_throttle(10.0, "cv_bridge 转换失败: %s", e)
            except Exception:
                pass

    def _gimbal_angle_callback(self, angle_info):
        try:
            pitch, yaw, _, _ = angle_info
        except (TypeError, ValueError):
            return

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        msg.position = [-yaw / 180.0 * math.pi, -pitch / 180.0 * math.pi]
        self._joint_pub.publish(msg)

    def _cmd_vel_callback(self, msg):
        if not hasattr(self, "ep_robot") or self.ep_robot is None:
            return

        self._last_cmd_time = rospy.Time.now()

        vx = msg.linear.x
        vy = msg.linear.y
        vz_rad = -msg.angular.z
        vz_deg = vz_rad * 180.0 / math.pi

        max_v = 2.0
        vx = max(-max_v, min(max_v, vx))
        vy = max(-max_v, min(max_v, vy))
        vz_deg = max(-360.0, min(360.0, vz_deg))

        try:
            self.ep_robot.chassis.drive_speed(
                x=vy, y=vx, z=vz_deg,
                timeout=self.cmd_vel_timeout
            )
        except Exception as e:
            rospy.logwarn_throttle(5.0, "发送速度指令失败: %s", e)

    def _check_cmd_vel_timeout(self, event):
        if not self.enable_cmd_vel or not hasattr(self, "ep_robot"):
            return

        elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
        if elapsed > self.cmd_vel_timeout:
            try:
                self.ep_robot.chassis.drive_speed(x=0, y=0, z=0, timeout=0)
            except Exception:
                pass

    def _safe_call(self, func, *args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            rospy.logwarn("调用 %r 失败: %s", func.__name__, e)
            return None

    def shutdown(self):
        rospy.loginfo("rm_ep_driver 正在关闭...")
        if hasattr(self, "ep_robot") and self.ep_robot is not None:
            try:
                if self._enable_camera:
                    self.ep_robot.camera.stop_video_stream()
                self.ep_robot.chassis.drive_speed(x=0, y=0, z=0, timeout=0)
                self.ep_robot.close()
            except Exception:
                pass
        rospy.loginfo("rm_ep_driver 已关闭")


if __name__ == "__main__":
    driver = None
    try:
        driver = RmEpDriver()
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("驱动节点异常退出: %s", e)
        if driver is not None:
            driver.shutdown()

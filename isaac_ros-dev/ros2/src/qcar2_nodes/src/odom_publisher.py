#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist, PoseWithCovarianceStamped
import tf2_ros
import math

class QCar2SensitiveOdom(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # 1. 物理常数 (源自硬件手册 P11)
        # 转换系数 = (1/2880 PPR) * 0.01977 (官方修正因子)
        self.CONV_COEFF = (1.0 / 2880.0) * 0.0217

        # 2. 状态变量 (支持从 /initialpose 同步)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.raw_vel = 0.0
        self.raw_gyro = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        
        # 3. 滤波参数：解决雷达轮廓抖动
        # alpha 越小越丝滑。0.08 能够大幅抑制仿真环境中的传感器毛刺
        self.smooth_omega = 0.0
        self.lpf_alpha = 0.5

        # 4. 校准参数
        self.vel_bias, self.gyro_bias = 0.0, 0.0
        self.is_calibrated = False
        self.samples_vel = []
        self.samples_gyro = []

        # 5. 纠偏微调 (注意：此处量级应在 0.001 左右，切勿设置如 -0.1 这样的大数)
        self.manual_yaw_trim = -0.0012 

        # 6. 通信句柄
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 订阅话题
        self.create_subscription(JointState, '/qcar2_joint', self.joint_cb, 10)
        self.create_subscription(Imu, '/qcar2_imu', self.imu_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # 🚀 解决“初始偏差”：订阅 RViz 的位姿设定工具
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_cb, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.02, self.update_loop) # 50Hz
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('✅ 敏感型对齐里程计已启动。可用 RViz "2D Pose Estimate" 对齐位置。')

    def initial_pose_cb(self, msg):
        """当你在 RViz 中点击 2D Pose Estimate 并拖动方向时，同步 TF 位置"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # 四元数转 Yaw
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self.get_logger().info(f'📍 坐标同步成功：x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')

    def cmd_cb(self, msg):
        """实时捕获控制指令"""
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def joint_cb(self, msg):
        if len(msg.velocity) > 0:
            self.raw_vel = msg.velocity[0]
            if not self.is_calibrated: self.samples_vel.append(self.raw_vel)

    def imu_cb(self, msg):
        self.raw_gyro = msg.angular_velocity.z
        if not self.is_calibrated: self.samples_gyro.append(self.raw_gyro)

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # --- 自动校准逻辑 ---
        if not self.is_calibrated:
            if len(self.samples_vel) >= 100:
                self.vel_bias = sum(self.samples_vel) / 100
                self.gyro_bias = sum(self.samples_gyro) / 100
                self.is_calibrated = True
                self.get_logger().info(f'📡 校准完成。速度基准:{self.vel_bias:.1f}')
            return

        # --- 核心纠偏逻辑：针对微调操作优化 ---
        has_linear_cmd = abs(self.cmd_linear_x) > 0.001
        has_angular_cmd = abs(self.cmd_angular_z) > 0.001 # 极低阈值，捕捉微调

        if not (has_linear_cmd or has_angular_cmd):
            # 完全静止：锁死所有漂移
            v, omega = 0.0, 0.0
            self.smooth_omega = 0.0
        else:
            # 计算基础线速度
            v = (self.raw_vel - self.vel_bias) * self.CONV_COEFF
            
            # 处理角速度
            if not has_angular_cmd:
                # 场景：你只按了前进。此时强制锁定航向，解决“直行偏左”
                current_omega = 0.0
            else:
                # 场景：你正在按左/右微调。释放控制权，完全信任 IMU
                current_omega = (self.raw_gyro - self.gyro_bias) + self.manual_yaw_trim
            
            # 强力一阶低通滤波：消除雷达轮廓剧烈抖动的元凶
            self.smooth_omega = (self.lpf_alpha * current_omega) + ((1.0 - self.lpf_alpha) * self.smooth_omega)
            omega = self.smooth_omega
            # --- 新增：角速度死区过滤 ---
            # 只要检测到的旋转量小于 0.03 rad/s，就强制视为直线行驶
            # 这能防止 TF 标志在直道上逐渐向左漂移，从而消除导航开始时的“右偏补偿”动作
            # if abs(omega) < 0.015:
            #     omega = 0.0

        # --- 中点法积分 (提升圆弧轨迹精度) ---
        delta_yaw = omega * dt
        self.x += v * math.cos(self.yaw + delta_yaw / 2.0) * dt
        self.y += v * math.sin(self.yaw + delta_yaw / 2.0) * dt
        self.yaw += delta_yaw

        self.publish_data(now, v, omega)

    def publish_data(self, now, v, omega):
        stamp = now.to_msg()
        q = Quaternion()
        q.w, q.z = math.cos(self.yaw/2), math.sin(self.yaw/2)
        
        # TF 发布：odom -> base_link
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.rotation = self.x, self.y, q
        self.tf_broadcaster.sendTransform(t)

        # Odom 话题发布
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.orientation = self.x, self.y, q
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, omega
        self.odom_pub.publish(o)

def main():
    rclpy.init(); rclpy.spin(QCar2SensitiveOdom()); rclpy.shutdown()

if __name__ == '__main__':
    main()
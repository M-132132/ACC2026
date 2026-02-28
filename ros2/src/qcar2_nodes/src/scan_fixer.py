#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        
        # 1. 匹配 QCar 原始雷达的 QoS (保持 Best Effort 以降低网络负担)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅原始雷达
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, qos_profile)
        # 发布修正后的雷达
        self.publisher = self.create_publisher(LaserScan, '/scan_correct', 10)
        
        # 2. 频率控制变量 (限制在 20Hz 左右，防止虚拟机 CPU 爆表)
        self.last_pub_time = self.get_clock().now()
        
        # 移除 TF 广播器，由 Launch 文件中的 static_transform_publisher 代替
        self.get_logger().info('✅ ScanFixer 优化版启动：专职处理点云数据，TF 已交由静态发布器接管')

    def listener_callback(self, msg):
        now = self.get_clock().now()
        
        # --- 频率控制逻辑 ---
        # 强制将处理频率限制在 ~20Hz (50ms 间隔)
        diff = (now - self.last_pub_time).nanoseconds
        if diff < 50000000: # 50ms
            return
            
        self.last_pub_time = now
        
        # --- A. 核心操作：重打时间戳 ---
        # 必须使用当前最新的时钟时间，否则 Nav2 会因为时间过时而拒绝处理扫描数据数据数据
        msg.header.stamp = now.to_msg()
        
        # --- B. 发布修正后的点云 ---
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
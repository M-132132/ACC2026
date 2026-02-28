import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WallStopper(Node):
    def __init__(self):
        super().__init__('wall_stopper')
        
        # 1. 订阅者：收听雷达数据 (/scan)
        # BEST_EFFORT 策略通常用于传感器数据，以防连接不稳定
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_policy)
            
        # 2. 发布者：发送控制指令 (/cmd_vel_nav)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        self.get_logger().info('🛡️ 防撞系统已启动！安全距离设定为: 1.0 米')

    def scan_callback(self, msg):
        # 这个函数每次收到雷达数据都会自动运行
        
        # 过滤掉无效数据（比如 inf 或 0.0）
        valid_ranges = [r for r in msg.ranges if r > 0.1 and r < 10.0]
        
        if not valid_ranges:
            return # 如果数据全是无效的，暂时不做判断
            
        # 找出最近的障碍物距离
        min_distance = min(valid_ranges)
        
        cmd_msg = Twist()
        
        # === 核心逻辑 ===
        if min_distance < 1.0:
            # 距离小于 1 米 -> 紧急停车！
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.get_logger().warning(f'🛑 危险！前方障碍物仅 {min_distance:.2f}米！已刹车。')
        else:
            # 距离安全 -> 慢慢前进
            cmd_msg.linear.x = 0.5  # 速度设慢点，安全第一
            cmd_msg.angular.z = 0.0
            self.get_logger().info(f'✅ 前方安全 ({min_distance:.2f}米)，继续前进...')
        # ===============
        
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallStopper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 退出时停车
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

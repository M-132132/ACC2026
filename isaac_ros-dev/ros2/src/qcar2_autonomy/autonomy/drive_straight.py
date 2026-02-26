import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveStraight(Node):
    def __init__(self):
        super().__init__('drive_straight_node')
        # 连接到“翻译官”
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # 设置定时器，每0.1秒执行一次 control_loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 记录开始时间
        self.start_time = time.time()
        self.duration = 5.0  # 设定直行时间：5秒
        
        self.get_logger().info('🚗 任务开始：准备直行 5 秒...')

    def control_loop(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        msg = Twist()

        # 判断是否在 5 秒内
        if elapsed_time < self.duration:
            # === 这里控制怎么走 ===
            msg.linear.x = 0.5   # 油门：1.0 m/s (前进)
            msg.angular.z = 0.0  # 转向：0.0 (不转弯，走直线)
            # ====================
            
            self.publisher_.publish(msg)
            remaining = self.duration - elapsed_time
            self.get_logger().info(f'直行中... 速度: {msg.linear.x}, 剩余时间: {remaining:.1f}s')
            
        else:
            # 超过 5 秒，发送停车指令
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('🛑 时间到！自动停车。')
            
            # 任务结束，关闭节点
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DriveStraight()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('main').info('程序已正常退出')
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
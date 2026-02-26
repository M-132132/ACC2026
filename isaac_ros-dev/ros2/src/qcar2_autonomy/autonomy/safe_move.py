import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # <--- 注意：改用通用速度消息，不用 MotorCommands 了

class SafeMover(Node):
    def __init__(self):
        super().__init__('safe_mover')
        # 发布到 '翻译官' 监听的频道
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('安全驾驶模式已启动...')

    def timer_callback(self):
        msg = Twist()
        
        # 1.0 m/s 前进 (修改这里可以改变速度)
        msg.linear.x = 1.0  
        # 0.5 rad/s 向左转 (修改这里可以改变方向)
        msg.angular.z = 1.0 
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'正在发送: 速度={msg.linear.x}, 转向={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = SafeMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 停车逻辑
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands # 导入官方定义的电机消息格式

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        # 创建发布者，直接向 /qcar2_motor_speed_cmd 发指令
        self.publisher_ = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', 10)
        
        # 设置定时器，每 0.1 秒发一次指令
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('老司机准备发车...')

    def timer_callback(self):
        msg = MotorCommands()
        # 设置控制通道：一个是油门，一个是转向
        msg.motor_names = ['motor_throttle', 'steering_angle']
        
        # 设置数值：油门 1.0 (m/s), 转向 0.0 (rad)
        # 你可以改这里的数字：正数前进，负数后退
        msg.values = [1.0, 0.0] 
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'发送指令: 油门={msg.values[0]}, 转向={msg.values[1]}')

def main(args=None):
    rclpy.init(args=args)
    mover = SimpleMover()
    
    try:
        rclpy.spin(mover)
    except KeyboardInterrupt:
        # 按 Ctrl+C 停车
        stop_msg = MotorCommands()
        stop_msg.motor_names = ['motor_throttle', 'steering_angle']
        stop_msg.values = [0.0, 0.0]
        mover.publisher_.publish(stop_msg)
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
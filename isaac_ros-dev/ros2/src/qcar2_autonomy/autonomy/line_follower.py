import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # ================= 配置区域 =================
        # 1. 摄像头话题 (已确认为 /camera/csi_image)
        self.camera_topic = '/camera/csi_image'
        
        # 2. 颜色阈值 (HSV空间中的黄色)
        # 如果发现识别不到，可能需要微调这里
        self.lower_yellow = np.array([20, 80, 80])
        self.upper_yellow = np.array([40, 255, 255])
        
        # 3. 速度设定
        self.base_speed = 0.2   # 直行速度 (m/s)
        self.turn_gain = 0.003  # 转向灵敏度 (Kp): 数值越大，转弯越猛
        # ===========================================

        # 订阅摄像头图像
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
            
        # 发布控制指令
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # 初始化 CV_Bridge
        self.bridge = CvBridge()
        
        self.get_logger().info(f'👁️ 视觉巡线已启动！正在监听: {self.camera_topic}')

    def image_callback(self, msg):
        try:
            # 1. 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 2. 截取感兴趣区域 (ROI)
        # 我们只看图像的下半部分，避免受远处背景干扰
        height, width, _ = cv_image.shape
        crop_img = cv_image[int(height/2):height, 0:width]
        
        # 3. 转换颜色空间 BGR -> HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # 4. 创建掩膜 (Mask): 只保留黄色像素
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # 5. 计算重心 (找到黄色的中心点)
        M = cv2.moments(mask)
        
        cmd_msg = Twist()
        
        if M['m00'] > 0:
            # --- 找到了黄线 ---
            cx = int(M['m10'] / M['m00']) # 黄色块的中心 X 坐标
            center_x = width // 2         # 屏幕中心 X 坐标
            
            # 计算误差: 目标在哪边？(正数在右，负数在左)
            error = cx - center_x
            
            # P控制器: 根据误差计算转向角度
            # 负号是因为: 误差为正(线在右边) -> 需要向右转(负Angular.z)
            cmd_msg.angular.z = -float(error) * self.turn_gain
            cmd_msg.linear.x = self.base_speed
            
            # (可选) 转弯时稍微减速，更稳
            if abs(error) > 50:
                cmd_msg.linear.x = 0.3

            self.get_logger().info(f'🟡 追踪中 | 误差: {error} | 转向: {cmd_msg.angular.z:.3f}')
            
        else:
            # --- 没看到黄线 ---
            # 策略: 原地慢速旋转寻找，或者停车
            self.get_logger().info('⚪ 丢失目标！搜索中...')
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.3 # 慢速左转找线
        
        # 发送指令
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
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

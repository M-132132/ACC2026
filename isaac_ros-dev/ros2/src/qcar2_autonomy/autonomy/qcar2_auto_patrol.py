#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

# 辅助函数：将角度（度）转换为四元数（四元数是 ROS 2 导航的标准格式）
def euler_to_quaternion(yaw_deg):
    yaw_rad = yaw_deg * (math.pi / 180.0)
    return [0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)]

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 1. 等待导航系统完全就绪
    # 这会检查 AMCL、Planner 和 Controller 是否都已激活
    navigator.waitUntilNav2Active()

    # 2. 定义 10 个巡航点（基于之前抓取的地图真值）
    # 格式: [x, y, yaw_degrees]
    # raw_goals = [
    #     [1.366, -0.307, -5.657],  # 点 1
    #     [2.221, -0.410, 0.797],   # 点 2
    #     [3.353, 0.085, 70.897],   # 点 3
    #     [3.626, 3.440, 84.838],   # 点 4
    #     [2.755, 4.766, 166.533],  # 点 5
    #     [0.786, 4.950, 179.901],  # 点 6
    #     [-0.218, 4.288, -101.361],# 点 7
    #     [-0.294, 2.696, -87.037], # 点 8
    #     [-0.343, 1.693, -84.880], # 点 9
    #     [0.167, 0.084, -42.945]   # 点 10
    # ]
    #[1.443, 3.598, 135.130],
    raw_goals = [
        [1.922, 1.187, 43.101],    # 点 1 [cite: 8]
        [2.396, 2.428, 114.441],   # 点 2 [cite: 14]
        [0.532, 4.885, 129.876],   # 点 3 [cite: 20]
        [-0.347, 5.827, 168.654],  # 点 4 [cite: 27]
        [-1.278, 5.389, -138.338], # 点 5 [cite: 32]
        [-2.638, 4.496, -133.306], # 点 6 [cite: 40]
        [-3.129, 3.322, -61.250],  # 点 7 [cite: 48]
        [-2.136, 1.951, -56.557],  # 点 8 [cite: 59]
        [-1.441, 1.049, -51.440],  # 点 9 [cite: 65]
        [-0.043, 0.121, 1.034]     # 点 10 [cite: 72]
    ]

    goal_poses = []
    for g in raw_goals:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = g[0]
        pose.pose.position.y = g[1]
        q = euler_to_quaternion(g[2])
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        goal_poses.append(pose)

    print(f"🚀 已加载 {len(goal_poses)} 个目标点，开始自动循环巡航...")

    # 3. 开启巡航循环
    while rclpy.ok():
        # 发送多路点任务
        navigator.followWaypoints(goal_poses)

        # 4. 实时监控进度
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                # 修正：使用 current_waypoint 获取当前目标点的索引
                # 我们把它加 1，让它从“第 1 个点”开始显示
                print(f'正在前往第 {feedback.current_waypoint + 1} 个目标点...', end='\r')

        # 5. 检查本轮结果
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('\n✨ 完成一圈巡航！休息 2 秒后开始下一圈...')
            navigator.get_clock().sleep_for(Duration(seconds=2.0))
        elif result == TaskResult.CANCELED:
            print('\n任务被取消')
            break
        elif result == TaskResult.FAILED:
            print('\n任务执行失败，正在尝试重新连接...')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
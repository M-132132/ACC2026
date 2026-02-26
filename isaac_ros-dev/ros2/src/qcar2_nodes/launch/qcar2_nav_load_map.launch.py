import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    qcar2_pkg = get_package_share_directory('qcar2_nodes')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # ================= 核心修正 =================
    # 使用你当前的真实绝对路径
    # 只要你的 my_final_map.yaml 在 ros2 文件夹下，这就一定能找到
    map_path = '/workspaces/isaac_ros-dev/ros2/map/my_best_map2.yaml'
    scan_fixer_script_path = '/workspaces/isaac_ros-dev/ros2/src/qcar2_nodes/src/scan_fixer.py'
    # 保持 false，这很重要
    sim_time_setting = 'false'
    # ===========================================

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=map_path,
        description='Full path to map file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=sim_time_setting,
        description='Use simulation (Gazebo) clock'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(qcar2_pkg, 'config', 'qcar2_slam_and_nav_virtual.yaml'),
        description='Nav2 parameters'
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 1. 启动小车驱动
    qcar2_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qcar2_pkg, 'launch', 'qcar2_virtual_launch.py')
        )
    )
    from launch_ros.actions import Node

    # 2. 静态 TF：base_link -> base_footprint
    static_tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'base_footprint']
    )

    # 3. 静态 TF：base_link -> base_scan（这里的 yaw 偏角用四元数写死最稳）
    # yaw = 0.17 rad -> qz=sin(yaw/2)=0.0849, qw=cos(yaw/2)=0.9964
    static_tf_base_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_scan',
        arguments=['0.1', '0', '0', '0', '0', '0', '1', 'base_link', 'base_scan']
    )

    # 2. 创建节点描述
    scan_fixer_node = Node(
        executable='python3',          # 指定运行环境
        arguments=[scan_fixer_script_path], # 指定脚本路径
        name='scan_fixer_node',
        output='screen'
    )
    

    # # 2. 启动雷达坐标系修正 (不能少)
    # lidar_tf_node = Node(
    #         package='qcar2_nodes',
    #         executable='fixed_lidar_frame_virtual',
    #         name='fixed_lidar_frame')
        
        
    
    # 3. 启动转换器
    converter_node = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
        output='screen',
        # ================= 核心修正：统一控制链路 =================
        # 强制转换器监听平滑后的信号 /cmd_vel，而不是原始的 /cmd_vel_nav
        # 这样硬件动作就会和里程计（Odom）完全同步，且受到平滑器的物理限制
        remappings=[
            ('/cmd_vel_nav', '/cmd_vel'),
            # 如果你的转换器代码里默认监听的是 /cmd_vel_nav，就写下面这一行：
            # ('/cmd_vel_nav', '/cmd_vel') 
        ]
        # ========================================================
    )
    
    # 4. 启动 Nav2 导航栈
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'slam': 'False'
        }.items()
    )



    # 强制发布一个绝对稳定的静态 TF，不走 scan_fixer，不走动态话题
    static_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_to_base_scan',
        # 参数依次是：x y z yaw pitch roll parent_frame child_frame
        # 根据你 echo 的数据：x=0.1, y=0, z=0
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'base_scan']
    )

    # return LaunchDescription([
    #     stdout_linebuf_envvar,
    #     map_file_arg,
    #     use_sim_time_arg,
    #     params_file_arg,
    #     qcar2_driver,
    #     static_tf_base_footprint,
    #     static_tf_base_scan,
    #     scan_fixer_node,
    #     converter_node,
    #     nav2_bringup
    # ])

    return LaunchDescription([
        stdout_linebuf_envvar,
        map_file_arg,
        use_sim_time_arg,
        params_file_arg,
        qcar2_driver,
        scan_fixer_node,
        static_tf_base_footprint,
        static_tf_base_scan, 
        static_lidar_tf_node,
        converter_node,
        nav2_bringup
    ])
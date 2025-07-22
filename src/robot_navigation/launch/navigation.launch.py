from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 设置日志格式
    log_config = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        '[{time}] [{name}] [{severity}] {message}'
    )

    # 声明参数
    map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            get_package_share_directory('robot_navigation'),
            'maps',
            'map.yaml'
        ),
        description='Path to the map file (YAML)'
    )

    map_topic = DeclareLaunchArgument('map_topic', default_value='map')
    start_pose_topic = DeclareLaunchArgument('start_pose_topic', default_value='/start_pose')
    goal_pose_topic = DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose')
    inflation_radius = DeclareLaunchArgument('inflation_radius', default_value='0.3')
    obstacle_threshold = DeclareLaunchArgument('obstacle_threshold', default_value='127.0')
    planning_frequency = DeclareLaunchArgument('planning_frequency', default_value='1.0')

    # 地图服务器节点（生命周期）
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='', 
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'frame_id': 'map',
            'topic_name': LaunchConfiguration('map_topic'),
            'use_sim_time': False
        }]
    )


    # 导航节点
    navigation_node = Node(
        package='robot_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'inflation_radius': LaunchConfiguration('inflation_radius'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'planning_frequency': LaunchConfiguration('planning_frequency')
        }],
        remappings=[
            ('map', LaunchConfiguration('map_topic')),
            ('start_pose', LaunchConfiguration('start_pose_topic')),
            ('goal_pose', LaunchConfiguration('goal_pose_topic'))
        ]
    )

    # RViz2 可视化配置
    rviz_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'config.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 静态 TF
    static_tf_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        output='screen'
    )

    static_tf_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher2',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom', '--child-frame-id', 'base_link'
        ],
        output='screen'
    )

    return LaunchDescription([
        log_config,
        map_file,
        map_topic,
        start_pose_topic,
        goal_pose_topic,
        inflation_radius,
        obstacle_threshold,
        planning_frequency,
        map_server_node,
        navigation_node,
        rviz_node,
        static_tf_node1,
        static_tf_node2
    ])

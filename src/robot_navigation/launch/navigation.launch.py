from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 设置日志格式
    log_config = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        '[{time}] [{name}] [{severity}] {message}'
    )

    # 使用正确的绝对路径
    map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            os.path.expanduser('~/Documents/workspace/gitProject/RosProject/ros2-navigation/src/robot_navigation'),
            'maps',
            'map.yaml'
        ),
        description='Path to the map file (YAML)'
    )

    map_topic = DeclareLaunchArgument('map_topic', default_value='map')
    start_pose_topic = DeclareLaunchArgument('start_pose_topic', default_value='/initialpose')
    goal_pose_topic = DeclareLaunchArgument('goal_pose_topic', default_value='goal_pose')
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

    # 地图服务器状态转换事件处理
    map_server_configure_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == map_server_node,
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    # 启动时配置地图服务器
    configure_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == map_server_node,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
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
        }]
    )

    force_x11 = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    # RViz2 可视化配置 - 使用正确路径
    rviz_config = os.path.join(
        os.path.expanduser('~/Documents/workspace/gitProject/RosProject/ros2-navigation/src/robot_navigation'),
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
        force_x11,
        map_file,
        map_topic,
        start_pose_topic,
        goal_pose_topic,
        inflation_radius,
        obstacle_threshold,
        planning_frequency,
        map_server_node,
        map_server_configure_event,
        configure_map_server,
        navigation_node,
        rviz_node,
        static_tf_node1,
        static_tf_node2
    ])
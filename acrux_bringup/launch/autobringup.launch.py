import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    firmware_dir         = os.path.join(get_package_share_directory('acrux_firmware'),    'launch')
    navigation_dir       = os.path.join(get_package_share_directory('acrux_navigation'),  'launch')
    rviz_launch_dir      = os.path.join(get_package_share_directory('acrux_description'), 'launch')
    gazebo_launch_dir    = os.path.join(get_package_share_directory('acrux_gazebo'),      'launch')
    slam_launch_dir      = os.path.join(get_package_share_directory('acrux_slam'),        'launch')
    map_directory        = os.path.join(get_package_share_directory('acrux_navigation'),  'maps', 'nav2_test_map.yaml')
    rviz_config_path     = os.path.join(get_package_share_directory('acrux_description'), 'rviz/navigation.rviz')
    x2_params_dir        = os.path.join(get_package_share_directory('acrux_firmware'),    'config', 'x2_params.yaml')
    robot_model_path     = os.path.join(get_package_share_directory('acrux_description'), 'urdf/acrux.xacro')

    map_file     = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration  = LaunchConfiguration('exploration')
    joy          = LaunchConfiguration('joy')
    slam         = LaunchConfiguration('slam')
    toolbox      = LaunchConfiguration('toolbox')
    realsense    = LaunchConfiguration('realsense')


    full_stack = PythonExpression(["str(", slam, ").lower() in ['false', '0']"])
    carto_mapping_active = PythonExpression(["str(", slam, ").lower() in ['false', '0'] and str(", toolbox, ").lower() in ['false', '0'] and str(", exploration, ").lower() in ['true', '1']"])
    carto_odom_active = PythonExpression(["str(", slam, ").lower() in ['false', '0'] and (str(", toolbox, ").lower() in ['true', '1'] or str(", exploration, ").lower() in ['false', '0'])"])
    carto_pure_slam = PythonExpression(["str(", slam, ").lower() in ['true', '1']"])

    rviz_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
        condition=IfCondition(use_sim_time),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rvizconfig': rviz_config_path
        }.items())

    state_publisher_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': robot_model_path
        }.items())

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
        condition=IfCondition(use_sim_time),
        launch_arguments={'use_sim_time': use_sim_time}.items())


    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
        condition=IfCondition(full_stack),
        launch_arguments={
            'exploration':  exploration,
            'map_file':     map_file,
            'use_sim_time': use_sim_time,
            'toolbox':      toolbox
        }.items())

    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'slam_toolbox.launch.py')),
        condition=IfCondition(PythonExpression(["str(", slam, ").lower() in ['false', '0'] and str(", toolbox, ").lower() in ['true', '1']"])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'exploration':  exploration,
            'map_file':     map_file
        }.items())

    cartographer_odom_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'cartographer.launch.py')),
        condition=IfCondition(carto_odom_active),
        launch_arguments={
            'exploration':  'False',
            'use_sim_time': use_sim_time
        }.items())

    cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'cartographer.launch.py')),
        condition=IfCondition(carto_mapping_active),
        launch_arguments={
            'exploration':  exploration,
            'use_sim_time': use_sim_time
        }.items())

    cartographer_pure_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'cartographer.launch.py')),
        condition=IfCondition(carto_pure_slam),
        launch_arguments={
            'exploration':  'True',
            'use_sim_time': use_sim_time
        }.items())


    only_ydlidar_launch_cmd = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        output='screen',
        emulate_tty=True,
        parameters=[x2_params_dir],
        namespace='/',
    )

    scan_filter_node = launch_ros.actions.Node(
        package='ydlidar_ros2_driver',
        executable='scan_filter_node',
        name='scan_filter',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        parameters=[{
            'range_min':        0.05,
            'range_max':        16.0,
            'jump_window':      30,
            'jump_thresh':      0.15,
            'min_cluster_rays': 8,
        }],
    )

    microros_node = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        arguments=['serial', '--dev', '/dev/esp', '-b', '921600'])

    network_status_node = launch_ros.actions.Node(
        package='acrux_firmware',
        executable='network_status_publisher_node',
        name='network_status_publisher',
        output='screen',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])))

    auto_joy_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir, 'auto_joy_teleop.launch.py')),
        condition=IfCondition(joy),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    realsense_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir, 'realsense_d435i.launch.py')),
        condition=IfCondition(realsense),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            name='use_sim_time', default_value='False',
            description='Use Gazebo simulation clock'
        ),
        DeclareLaunchArgument(
            name='exploration', default_value='True',
            description='True = mapping mode, False = localization/navigation mode'
        ),
        DeclareLaunchArgument(
            name='slam', default_value='False',
            description='True = pure Cartographer SLAM for mapping only (no Nav2). '
                        'False = full navigation stack.'
        ),
        DeclareLaunchArgument(
            name='toolbox', default_value='False',
            description='True = use SLAM Toolbox for map->odom + Cartographer odom->base_link. '
                        'False = Cartographer handles full SLAM or Nav2 AMCL for localization.'
        ),
        DeclareLaunchArgument(
            name='map_file', default_value=map_directory,
            description='Map YAML used in localization mode'
        ),
        DeclareLaunchArgument(
            name='joy', default_value='True',
            description='Enable joystick control'
        ),
        DeclareLaunchArgument(
            name='realsense', default_value='False',
            description='Realsense camera node'
        ),

        rviz_launch_cmd,
        state_publisher_launch_cmd,
        gazebo_launch_cmd,
        navigation_launch_cmd,
        slam_toolbox_launch_cmd,
        cartographer_odom_launch_cmd,
        cartographer_launch_cmd,
        cartographer_pure_slam_cmd,
        only_ydlidar_launch_cmd,
        scan_filter_node,
        microros_node,
        network_status_node,
        auto_joy_cmd,
        realsense_launch_cmd,
    ])

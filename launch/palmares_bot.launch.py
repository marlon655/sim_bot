import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction


def generate_launch_description():

    # Package name
    package_name = 'sim_bot'

    # Launch configurations
    read_world = LaunchConfiguration('world')
    read_headless = LaunchConfiguration('headless')
    read_rviz = LaunchConfiguration('rviz')
    read_joy = LaunchConfiguration('joy')
    read_slam = LaunchConfiguration('slam')
    read_nav = LaunchConfiguration('nav')

    # Path to default world
    world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'test.world')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Run Gazebo headless (no GUI) if set to True')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Open RViz if set to True')

    declare_joy = DeclareLaunchArgument(
        name='joy', default_value='True',
        description='Enable joystick tele-operation')

    declare_slam = DeclareLaunchArgument(
        name='slam', default_value='True',
        description='Enable SLAM (slam_toolbox)')

    declare_nav = DeclareLaunchArgument(
        name='nav', default_value='True',
        description='Enable Nav2 navigation stack')

    # ── Robot State Publisher ──────────────────────────────────────────────
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'description', 'palmares_bot.urdf.xacro')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # ── Joystick Tele-op (optional) ───────────────────────────────────────
    joystick = GroupAction(
        condition=IfCondition(read_joy),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'teleop.launch.py'
            )]),
            launch_arguments={'use_sim_time': 'true'}.items())]
    )

    # ── Twist Mux ─────────────────────────────────────────────────────────
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux_params.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # ── Gazebo server ─────────────────────────────────────────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', read_world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Gazebo client (GUI, skipped when headless=True) ───────────────────
    gazebo_client = GroupAction(
        condition=IfCondition(PythonExpression(['not ', read_headless])),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )]),
            launch_arguments={'gz_args': '-g '}.items())]
    )

    # ── Spawn palmares_bot ────────────────────────────────────────────────
    spawn_palmares_bot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'palmares_bot',
                   '-z', '0.15'],
        output='screen'
    )

    # ── Gazebo ↔ ROS bridge ───────────────────────────────────────────────
    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(read_rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            remappings=[
                ('/map', 'map'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/goal_pose', 'goal_pose'),
                ('/clicked_point', 'clicked_point'),
                ('/initialpose', 'initialpose'),
            ])]
    )

    # ── SLAM (optional) ───────────────────────────────────────────────────
    slam_node = GroupAction(
        condition=IfCondition(read_slam),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'slam.launch.py'
            )]),
            launch_arguments={'use_sim_time': 'true'}.items())]
    )

    # ── Nav2 (optional) ───────────────────────────────────────────────────
    nav_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'nav_params.yaml')
    nav_node = GroupAction(
        condition=IfCondition(read_nav),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'nav.launch.py'
            )]),
            launch_arguments={'use_sim_time': 'true', 'params_file': nav_params}.items())]
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_headless,
        declare_rviz,
        declare_joy,
        declare_world,
        declare_slam,
        declare_nav,

        # Launch nodes
        rviz2,
        rsp,
        twist_mux,
        joystick,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_palmares_bot,
        slam_node,
        nav_node,
    ])

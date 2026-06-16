import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, GroupAction,
    SetEnvironmentVariable, TimerAction)


def generate_launch_description():

    # Package name
    package_name = 'sim_bot'

    # ── Make Gazebo aware of the custom charging_dock model ───────────────
    # Append our models/ directory to GZ_SIM_RESOURCE_PATH so that
    # <include><uri>model://charging_dock</uri></include> in test.world
    # resolves correctly.  We prepend any pre-existing path.
    _models_path = os.path.join(
        get_package_share_directory(package_name), 'models')
    _existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    _gz_resource_path = (_existing_gz_path + ':' + _models_path
                         if _existing_gz_path else _models_path)
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', _gz_resource_path)

    # Launch configurations
    read_world = LaunchConfiguration('world')
    read_headless = LaunchConfiguration('headless')
    read_rviz = LaunchConfiguration('rviz')
    read_joy = LaunchConfiguration('joy')
    read_slam = LaunchConfiguration('slam')
    read_nav = LaunchConfiguration('nav')

    # Path to default world
    world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'factory.world')

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

    declare_dock = DeclareLaunchArgument(
        name='dock', default_value='True',
        description='Enable ArUco/LiDAR docking system')

    read_dock = LaunchConfiguration('dock')

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
    # Nasce no centro do galpão (world 0,0) = odom(0,0).
    # Dock em world(13,0) = odom(13,0). Staging em odom(12,0).
    # Robot faces +X (direção do dock na parede direita).
    spawn_palmares_bot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'palmares_bot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.15',
                   '-Y', '0.0'],
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

    # ── odom → TF relay (clean 30 Hz TF, avoids Gazebo "jump back in time") ──
    odom_to_tf = Node(
        package='sim_bot',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── joint_state_publisher (monotonic sim-time stamps, no Gazebo jitter) ──
    # Replaces the gz_bridge joint_states bridge. Publishes zero-position wheel
    # joints at 10 Hz using sim clock so robot_state_publisher never sees
    # "Moved backwards in time" and never flushes the TF buffer.
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
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
    # Delay 5 s so Gazebo is fully running and odom_to_tf is publishing TF
    # before slam_toolbox tries to process its first scan.  Without this delay
    # the very first scan may arrive before odom_to_tf has published any TF,
    # causing tf2::NoDataForExtrapolationException on startup.
    slam_node = TimerAction(
        period=5.0,
        actions=[GroupAction(
            condition=IfCondition(read_slam),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'slam.launch.py'
                )]),
                launch_arguments={'use_sim_time': 'true'}.items())]
        )]
    )

    # ── Nav2 (optional) ───────────────────────────────────────────────────
    nav_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'nav_params.yaml')
    # Delay Nav2 by 20 seconds so that Gazebo has time to start publishing
    # /scan, SLAM can receive it and begin publishing the map->odom TF,
    # before the global_costmap tries to look up that transform.
    nav_node = TimerAction(
        period=20.0,
        actions=[GroupAction(
            condition=IfCondition(read_nav),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'nav.launch.py'
                )]),
                launch_arguments={'use_sim_time': 'true', 'params_file': nav_params}.items())]
        )]
    )

    # ── Docking system (optional) ─────────────────────────────────────────
    dock_node = TimerAction(
        period=25.0,
        actions=[GroupAction(
            condition=IfCondition(read_dock),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'docking.launch.py'
                )])
            )]
        )]
    )

    return LaunchDescription([
        # Environment
        set_gz_resource_path,

        # Declare launch arguments
        declare_headless,
        declare_rviz,
        declare_joy,
        declare_world,
        declare_slam,
        declare_nav,
        declare_dock,

        # Launch nodes
        rviz2,
        rsp,
        twist_mux,
        joystick,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        odom_to_tf,
        joint_state_pub,
        spawn_palmares_bot,
        slam_node,
        nav_node,
        dock_node,
    ])

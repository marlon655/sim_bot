import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, AppendEnvironmentVariable


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
    read_map = LaunchConfiguration('map')
    read_params_file = LaunchConfiguration('params_file')

    # Path to default world
    world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'test.world')
    models_path = os.path.join(get_package_share_directory(package_name), 'models')

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

    declare_map = DeclareLaunchArgument(
        name='map', default_value='',
        description='Full path to a map YAML file. If set, Nav2 starts map_server and AMCL.')

    declare_params_file = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name), 'config', 'nav_params.yaml'),
        description='Full path to the Nav2 parameters file')

    gazebo_models_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path,
        separator=':')

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
                   '-x', '18.34406852722168',
                   '-y', '22.93574333190918',
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
    nav_node = GroupAction(
        condition=IfCondition(read_nav),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'nav.launch.py'
            )]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': read_params_file,
                'map': read_map,
            }.items())]
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_headless,
        declare_rviz,
        declare_joy,
        declare_world,
        declare_slam,
        declare_nav,
        declare_map,
        declare_params_file,
        gazebo_models_path,

        # Launch nodes
        rviz2,
        rsp,
        joystick,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_palmares_bot,
        slam_node,
        nav_node,
    ])

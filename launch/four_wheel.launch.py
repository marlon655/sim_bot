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
    package_name='sim_bot' 

    # Launch configurations
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    joy = LaunchConfiguration('joy')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    octomap = LaunchConfiguration('octomap')

    # Path to default world 
    world_path = os.path.join(get_package_share_directory(package_name),'worlds', 'test.world')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Decides if the simulation is visualized')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')
    
    declare_joy = DeclareLaunchArgument(
        name='joy', default_value='True',
        description='Activates joy tele-operation')
    
    declare_slam = DeclareLaunchArgument(
        name='slam', default_value='True',
        description='Activates simultaneous localization and mapping')
    
    declare_nav = DeclareLaunchArgument(
        name='nav', default_value='True',
        description='Activates the navigation stack')

    declare_octomap = DeclareLaunchArgument(
        name='octomap', default_value='False',
        description='Activates 3D mapping if set to true')

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(get_package_share_directory(package_name),'description','four_wheel.urdf.xacro')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )
    
    # Launch Joystick Tele Operation Node if activated (active by default)
    joystick = GroupAction(
        condition=IfCondition(joy),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','teleop.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true'}.items())]
    )

    # Launch the Twist Mux Node
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux_params.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Launch the gazebo client to visualize the simulation only if headless is declared as False
    gazebo_client = GroupAction(
        condition=IfCondition(PythonExpression(['not ', headless])),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': '-g '}.items())]
    )

    # Run the spawner node from the gazebo_ros package. 
    spawn_diff_bot = Node(
                        package='ros_gz_sim', 
                        executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'four_wheel_bot',
                                   '-z', '0.2'],
                        output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"]
    )
    
    # Launch Rviz with diff bot rviz file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',
                    remappings=[('/map', 'map'),
                                ('/tf', 'tf'),
                                ('/tf_static', 'tf_static'),
                                ('/goal_pose', 'goal_pose'),
                                ('/clicked_point', 'clicked_point'),
                                ('/initialpose', 'initialpose')])]
    )

    # Launch Simultaneous Localization and Mapping
    slam_node = GroupAction(
        condition=IfCondition(slam),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','slam.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true'}.items())]
    )

    # Launch the navigation stack
    nav_params = os.path.join(get_package_share_directory(package_name), 'config', 'nav_params.yaml')
    nav_node = GroupAction(
        condition=IfCondition(nav),
        actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','nav.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true', 'params_file': nav_params}.items())]
    )

    # Launch 3D mapping stack 
    octomap_node = GroupAction(
        condition=IfCondition(octomap),
        actions=[Node(package='octomap_server',
                      executable='octomap_server_node',
                      name='octomap_server',
                      output='screen',
                      parameters=[{'resolution': 0.05,
                                   'frame_id': 'odom',
                                   'sensor_model.max_range': 5.0}],
                      remappings=[('/cloud_in','/camera/points')])],
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_headless,
        declare_rviz,
        declare_joy,
        declare_world,
        declare_slam,
        declare_nav,
        declare_octomap,

        # Launch the nodes
        rviz2,
        rsp,
        twist_mux,
        joystick,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        ros_gz_image_bridge,
        spawn_diff_bot,
        slam_node,
        nav_node,
        octomap_node
    ])

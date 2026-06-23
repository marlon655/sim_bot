"""
palmares_bot_real.launch.py — sistema completo para hardware real.

Diferenças em relação ao palmares_bot.launch.py (simulação):
  - SEM Gazebo (sem gz_sim, ros_gz_bridge, spawn_entity)
  - use_sim_time: False em todos os nós
  - Câmera: usb_cam via camera_real.launch.py (não gz_bridge)
  - TF odom→base_footprint: vem do hardware real via odom_to_tf.py
  - Sem TimerAction — hardware não precisa aguardar Gazebo subir

Pré-requisitos de hardware:
  - /odom publicado pelo controlador de encoder
  - /scan publicado pelo LiDAR
  - /cmd_vel lido pelo controlador de motores
  - Câmera C270 em /dev/camera_c270
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('sim_bot')

    declare_rviz = DeclareLaunchArgument('rviz',  default_value='True')
    declare_slam = DeclareLaunchArgument('slam',  default_value='True')
    declare_nav  = DeclareLaunchArgument('nav',   default_value='True')
    declare_dock = DeclareLaunchArgument('dock',  default_value='True')

    read_rviz = LaunchConfiguration('rviz')
    read_slam = LaunchConfiguration('slam')
    read_nav  = LaunchConfiguration('nav')
    read_dock = LaunchConfiguration('dock')

    urdf_path = os.path.join(pkg, 'description', 'palmares_bot.urdf.xacro')

    # ── Robot State Publisher ──────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false', 'urdf': urdf_path}.items()
    )

    # ── odom → TF relay (converte /odom do encoder para TF odom→base_footprint) ──
    odom_to_tf = Node(
        package='sim_bot',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ── joint_state_publisher (TF das rodas) ──────────────────────────────
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── Câmera USB C270 ────────────────────────────────────────────────────
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'camera_real.launch.py'))
    )

    # ── Twist Mux ─────────────────────────────────────────────────────────
    twist_mux_params = os.path.join(pkg, 'config', 'twist_mux_params.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(read_rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )]
    )

    # ── SLAM ──────────────────────────────────────────────────────────────
    slam_node = GroupAction(
        condition=IfCondition(read_slam),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()
        )]
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────
    nav_params = os.path.join(pkg, 'config', 'nav_params.yaml')
    nav_node = GroupAction(
        condition=IfCondition(read_nav),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file':  nav_params
            }.items()
        )]
    )

    # ── Docking ────────────────────────────────────────────────────────────
    docking = GroupAction(
        condition=IfCondition(read_dock),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'docking_real.launch.py'))
        )]
    )

    return LaunchDescription([
        declare_rviz,
        declare_slam,
        declare_nav,
        declare_dock,

        rsp,
        odom_to_tf,
        joint_state_pub,
        camera,
        twist_mux,
        rviz2,
        slam_node,
        nav_node,
        docking,
    ])

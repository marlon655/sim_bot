import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('sim_bot')
    docking_params = os.path.join(pkg, 'config', 'docking_params.yaml')

    # ── Dock pose estimator (ArUco Phase 1 + LiDAR Phase 2) ──────────────
    dock_estimator = Node(
        package='sim_bot',
        executable='dock_pose_estimator.py',
        name='dock_pose_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_id':       771,
            'marker_size':     0.30,    # physical marker size [m] (outer black border is 300 mm on a 400 mm panel)
            'stop_distance':   0.32,    # offset before marker → ~5 cm clearance
            'aruco_timeout':   1.5,     # s – LiDAR takes over after this
            'sector_half_deg': 30.0,    # LiDAR frontal sector half-angle
        }]
    )

    # ── opennav DockingServer ─────────────────────────────────────────────
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[docking_params, {'use_sim_time': True}],
        remappings=[
            ('/tf',        '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Lifecycle manager to auto-configure and activate the docking server
    docking_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['docking_server'],
        }]
    )

    return LaunchDescription([
        dock_estimator,
        docking_server,
        docking_lifecycle_manager,
    ])

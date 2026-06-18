import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('sim_bot')
    docking_params = os.path.join(pkg, 'config', 'docking_params.yaml')

    # ── Dock pose estimator (LiDAR-only) ─────────────────────────────────
    dock_estimator = Node(
        package='sim_bot',
        executable='dock_pose_estimator.py',
        name='dock_pose_estimator',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'stop_distance':     0.42,   # test
            'sector_half_deg':   30.0,   # normal frontal sector half-angle
            'close_sector_deg':  60.0,   # wide sector for lateral edge detection at close range
            'close_range_m':     0.5,    # switch to wide sector below this range
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

    # ── Charging manager (full autonomous charge cycle orchestrator) ───────
    charging_manager = Node(
        package='sim_bot',
        executable='charging_manager.py',
        name='charging_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        dock_estimator,
        docking_server,
        docking_lifecycle_manager,
        charging_manager,
    ])

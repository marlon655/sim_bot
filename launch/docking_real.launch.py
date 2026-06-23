"""
docking_real.launch.py — sistema de docking para hardware real.

Diferenças em relação ao docking.launch.py (simulação):
  - use_sim_time: False em todos os nós
  - Ajustar stop_distance e dock pose para o dock físico real
"""

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
            'use_sim_time':      False,      # CRÍTICO: False no hardware real
            'marker_id':         771,
            'marker_size':       0.15,       # medir o marcador físico impresso [m]
            'stop_distance':     0.30,       # calibrar empiricamente (gap ~5cm)
            'aruco_timeout':     2.0,
            'sector_half_deg':   30.0,
            'close_sector_deg':  60.0,
            'close_range_m':     0.5,
        }]
    )

    # ── opennav DockingServer ─────────────────────────────────────────────
    # ATENÇÃO: atualizar docking_params.yaml com a pose real do dock físico
    # antes de usar este launch file (campo "pose" em base_carregamento)
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[docking_params, {'use_sim_time': False}],
        remappings=[
            ('/tf',        '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    docking_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['docking_server'],
        }]
    )

    # ── Charging manager ──────────────────────────────────────────────────
    charging_manager = Node(
        package='sim_bot',
        executable='charging_manager.py',
        name='charging_manager',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        dock_estimator,
        docking_server,
        docking_lifecycle_manager,
        charging_manager,
    ])

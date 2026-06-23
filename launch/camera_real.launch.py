import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('sim_bot')
    cam_params = os.path.join(pkg, 'config', 'usb_cam_params.yaml')

    # Resolve o caminho da calibração sem hardcodar o usuário
    home = os.path.expanduser('~')
    calib_url = f'file://{home}/.ros/camera_info/c270.yaml'

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            cam_params,
            {'camera_info_url': calib_url},    # sobrescreve o caminho do YAML
        ],
        remappings=[
            # dock_pose_estimator assina /camera/image e /camera/camera_info
            ('/image_raw',   '/camera/image'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )

    return LaunchDescription([camera_node])

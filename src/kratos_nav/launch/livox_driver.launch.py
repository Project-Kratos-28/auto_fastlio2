"""MID-360 driver only, PointCloud2 (xfer_format 0), no RViz window.

Same settings as livox_ros_driver2's rviz_MID360_launch.py, minus the extra
rviz2 that file starts. Use the single RViz from config/kratos_live.rviz instead.

The MID-360's IP depends on the unit (192.168.1.162, .125, ...). This launch finds
the one that is plugged in (lidar_ip.py) and hands the driver a copy of
MID360_config.json with that address. The repo file is never edited. Force an
address with LIVOX_LIDAR_IP=192.168.1.xxx if the search picks the wrong device.

Only ONE driver may run: a second one binds the same UDP ports and both go silent.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lidar_ip  # noqa: E402  (sibling module, found via the path line above)


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('livox_ros_driver2'),
                       'config', 'MID360_config.json')
    ip = lidar_ip.find_lidar_ip()
    if ip:
        cfg = lidar_ip.write_config(
            cfg, f'/tmp/kratos_MID360_config_{os.getuid()}.json', ip)
        print(f'[livox_driver.launch] MID-360 found at {ip}', flush=True)
    else:
        print('[livox_driver.launch] WARNING: no LiDAR answered on 192.168.1.1xx - '
              'check power (it needs ~20 s to boot), cable, and that this host has '
              '192.168.1.10. Falling back to the IP written in MID360_config.json.',
              flush=True)
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 0,        # PointCloud2, needed by GLIM and lidar_angle_filter
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '/home/livox/livox_test.lvx',
                'user_config_path': cfg,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),
    ])

"""Nav2 on the live GLIM map.

Starts the two things Nav2 needs that nobody else publishes:
  1. static TF  base_link -> livox_frame  (where the LiDAR sits on the rover)
  2. Nav2 itself (planner, controller, behaviors, bt_navigator, waypoint_follower,
     velocity_smoother) through nav2_bringup's navigation_launch.py.

Deliberately NOT started: map_server (pcd2pgm publishes /map live) and AMCL
(GLIM publishes map->odom). Driver, angle filter, GLIM and pcd2pgm run in their
own terminals; see docs/subsystem/LIVE_MISSION_TEST.md (background/design: NAV2_BRINGUP.md).

Run:  ros2 launch kratos_nav nav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('kratos_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')

    # BT xml paths have to be absolute, and they differ per machine, so fill them
    # in at launch time instead of hardcoding them in the yaml. BOTH must be set:
    # the stock through-poses tree calls Spin, which we don't load, and
    # bt_navigator refuses to activate if any tree references a missing server.
    bt_dir = os.path.join(pkg, 'behavior_trees')
    rewritten_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_no_spin.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'navigate_through_poses_no_spin.xml'),
        },
        convert_types=True,
    )

    # PLACEHOLDER MOUNT. base_link is defined ON THE GROUND directly under the
    # rover's turning centre. lidar_x = how far forward of that the MID-360 is,
    # lidar_z = MID-360 height above the ground (tape measure to the centre of
    # the sensor). lidar_z also fixes where "ground" is for every height filter
    # (nav2_params.yaml obstacle heights, pcd2pgm_live.yaml Z band).
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox',
        arguments=['--x', LaunchConfiguration('lidar_x'),
                   '--y', '0.0',
                   '--z', LaunchConfiguration('lidar_z'),
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'livox_frame'],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': rewritten_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.60'),
        lidar_tf,
        nav2,
    ])

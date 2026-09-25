"""Whole Kratos rover stack in one command (run inside the kratos_glim container).

  ros2 launch kratos_bringup kratos.launch.py                      # everything, ESS depth, RViz
  ros2 launch kratos_bringup kratos.launch.py depth:=zed           # ZED SDK NEURAL depth instead of ESS
  ros2 launch kratos_bringup kratos.launch.py depth:=none          # LiDAR only (no camera, no nvblox)
  ros2 launch kratos_bringup kratos.launch.py gui:=false           # headless (default when no local display)
  ros2 launch kratos_bringup kratos.launch.py lidar_z:=0.62 cam_x:=0.35 cam_z:=0.45 cam_pitch:=0.30

From the host, ~/kratos_glim/start.sh does the container part and passes arguments through.

Starts, in this order:
  1. livox_ros_driver2          /livox/lidar (PointCloud2, 10 Hz), /livox/imu (200 Hz)
  2. lidar_angle_filter         /livox/lidar_filtered (antenna sectors masked) for GLIM
  3. kratos_nav nav.launch.py   static TF base_link -> livox_frame, base_link -> zed_camera_link; Nav2
  4. GLIM (after 3 s, once the static TF exists)   TF map -> odom -> base_link, /glim_ros/map
     KEEP THE ROVER STILL until GLIM logs "initial IMU state estimation result" (~5 s).
  5. pcd2pgm (live)             /glim_ros/map -> /map for the global costmap
  6. kratos_perception          ZED 2i -> ESS (or NEURAL) -> nvblox -> local costmap nvblox_layer
  7. RViz                       rviz/kratos.rviz (only with gui)

gui:=auto (default) turns the GUIs on only when DISPLAY is a local X display (":1"), i.e. not over
SSH (unset, or "localhost:10.0" with X forwarding, which is far too slow for OpenGL over a radio).
Without gui, RViz is not started and GLIM runs without its OpenGL viewer: GLIM crashes at start
when that viewer cannot open a display ("failed to initialize GLFW", SIGSEGV). Its ROS map
publisher (librviz_viewer.so, /glim_ros/map for pcd2pgm) is kept. From the laptop, view the rover
with rviz/laptop.rviz instead (light topics only).

Not started: the mission (waypoint_mission.py). The stack ends at Nav2's /cmd_vel.
lidar_z is passed to nav.launch.py AND kratos_perception, so this is the one place to set it;
nav2_params.yaml and pcd2pgm_live.yaml still carry the matching height numbers (AGENTS.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

WS = '/workspaces/kratos_glim'


def share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def gui_enabled(context):
    gui = context.launch_configurations['gui'].lower()
    if gui == 'auto':
        return os.environ.get('DISPLAY', '').startswith(':')
    return gui == 'true'


def glim_and_rviz(context):
    """GLIM with or without its OpenGL viewer, and RViz, depending on gui."""
    gui = gui_enabled(context)
    config = os.path.join(WS, 'glim', 'glim_config')
    if not gui:
        # Same config minus the desktop viewer (the JSON files have comments: edit as text).
        headless = f'/tmp/kratos_glim_config_headless_{os.getuid()}'
        os.makedirs(headless, exist_ok=True)
        for name in os.listdir(config):
            with open(os.path.join(config, name)) as f:
                text = f.read()
            if name == 'config_ros.json':
                text = '\n'.join(line for line in text.split('\n') if '"libstandard_viewer.so"' not in line)
            with open(os.path.join(headless, name), 'w') as f:
                f.write(text)
        config = headless
    actions = [TimerAction(period=3.0, actions=[Node(
        package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
        parameters=[{'config_path': config}])])]
    if gui:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', share('kratos_bringup', 'rviz', 'kratos.rviz')]))
    return actions


def generate_launch_description():
    arg = LaunchConfiguration
    mount_args = ['lidar_x', 'lidar_z', 'cam_x', 'cam_y', 'cam_z', 'cam_pitch', 'cam_yaw']

    driver = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,          # PointCloud2 (GLIM needs it; 1 = Livox CustomMsg)
            'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0, 'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': share('livox_ros_driver2', 'config', 'MID360_config.json'),
        }])
    angle_filter = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share('lidar_angle_filter', 'launch', 'angle_filter.launch.py')))
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('kratos_nav', 'launch', 'nav.launch.py')),
        launch_arguments={a: arg(a) for a in mount_args}.items())
    pcd2pgm = Node(
        package='pcd2pgm', executable='pcd2pgm_node', name='pcd2pgm', output='screen',
        parameters=[share('pcd2pgm', 'config', 'pcd2pgm_live.yaml')])
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('kratos_perception', 'launch', 'perception.launch.py')),
        launch_arguments={'depth': arg('depth'), 'lidar_z': arg('lidar_z')}.items(),
        condition=UnlessCondition(PythonExpression(["'", arg('depth'), "' == 'none'"])))

    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='ess', description='ess | zed | none'),
        DeclareLaunchArgument('gui', default_value='auto',
                              description='auto (on only with a local X display) | true | false'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='MID-360 forward of base_link (m)'),
        DeclareLaunchArgument('lidar_z', default_value='0.60', description='MID-360 height above ground (m)'),
        DeclareLaunchArgument('cam_x', default_value='0.30', description='ZED forward of base_link (m), PLACEHOLDER'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.45', description='ZED height above ground (m), PLACEHOLDER'),
        DeclareLaunchArgument('cam_pitch', default_value='0.30', description='ZED down-tilt (rad), PLACEHOLDER'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        driver, angle_filter, nav, pcd2pgm, perception, OpaqueFunction(function=glim_and_rviz),
    ])

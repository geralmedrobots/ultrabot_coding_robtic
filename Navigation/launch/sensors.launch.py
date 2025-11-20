#!/usr/bin/env python3
"""Launch RealSense D455 and Ouster 3D LiDAR drivers with optional RViz visualization."""

import os

from ament_index_python.packages import get_package_share_directory  # type: ignore[import-not-found]
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # type: ignore[import-not-found]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('somanet')
    realsense_config = os.path.join(pkg_share, 'config', 'realsense_d455.yaml')
    ouster_config = os.path.join(pkg_share, 'config', 'ouster_lidar.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'sensor_visualization.rviz')

    enable_realsense = LaunchConfiguration('enable_realsense')
    enable_ouster = LaunchConfiguration('enable_ouster')
    enable_rviz = LaunchConfiguration('enable_rviz')
    realsense_serial = LaunchConfiguration('realsense_serial')
    ouster_hostname = LaunchConfiguration('ouster_hostname')
    ouster_metadata = LaunchConfiguration('ouster_metadata')
    ouster_udp_dest = LaunchConfiguration('ouster_udp_dest')
    ouster_lidar_mode = LaunchConfiguration('ouster_lidar_mode')
    ouster_timestamp_mode = LaunchConfiguration('ouster_timestamp_mode')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='realsense',
        name='realsense2_camera',
        output='screen',
        parameters=[
            realsense_config,
            {'serial_no': realsense_serial},
        ],
        condition=IfCondition(enable_realsense),
    )

    ouster_sensor_node = Node(
        package='ouster_ros',
        executable='os_sensor',
        namespace='ouster',
        name='os_sensor',
        output='screen',
        parameters=[
            ouster_config,
            {  
                'sensor_hostname': ouster_hostname,
                'metadata': ouster_metadata,
                'udp_dest': ouster_udp_dest,
                'lidar_mode': ouster_lidar_mode,
                'timestamp_mode': ouster_timestamp_mode,
            },
        ],
        condition=IfCondition(enable_ouster),
    )

    ouster_cloud_node = Node(
        package='ouster_ros',
        executable='os_cloud',
        namespace='ouster',
        name='os_cloud',
        output='screen',
        parameters=[
            ouster_config,
            {  
                'sensor_hostname': ouster_hostname,
                'metadata': ouster_metadata,
                'udp_dest': ouster_udp_dest,
                'lidar_mode': ouster_lidar_mode,
                'timestamp_mode': ouster_timestamp_mode,
            },
        ],
        condition=IfCondition(enable_ouster),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sensor_visualizer',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_realsense',
            default_value='true',
            description='Launch the Intel RealSense D455 driver.',
        ),
        DeclareLaunchArgument(
            'enable_ouster',
            default_value='true',
            description='Launch the Ouster OS-series LiDAR driver.',
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Launch RViz with a preconfigured sensor visualization.',
        ),
        DeclareLaunchArgument(
            'realsense_serial',
            default_value='',
            description='Optional RealSense serial number to bind to a specific camera.',
        ),
        DeclareLaunchArgument(
            'ouster_hostname',
            default_value='os-1.local',
            description='Hostname or IP address of the Ouster LiDAR.',
        ),
        DeclareLaunchArgument(
            'ouster_metadata',
            default_value='/tmp/ouster_metadata.json',
            description='Path to the metadata file exported from the Ouster sensor.',
        ),
        DeclareLaunchArgument(
            'ouster_udp_dest',
            default_value='auto',
            description='Destination IP for UDP packets ("auto" uses the machine running this launch file).',
        ),
        DeclareLaunchArgument(
            'ouster_lidar_mode',
            default_value='1024x10',
            description='LiDAR scan pattern (e.g., 512x20, 1024x10).',
        ),
        DeclareLaunchArgument(
            'ouster_timestamp_mode',
            default_value='TIME_FROM_INTERNAL_OSC',
            description='Timestamp source used by the Ouster sensor.',
        ),
        realsense_node,
        ouster_sensor_node,
        ouster_cloud_node,
        rviz_node,
    ])

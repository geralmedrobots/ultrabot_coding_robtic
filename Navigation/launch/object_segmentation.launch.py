#!/usr/bin/env python3
"""Launch object segmentation and detection pipeline for depth sensors."""

import os

from ament_index_python.packages import get_package_share_directory  # type: ignore[import-not-found]
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # type: ignore[import-not-found]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('somanet')
    params_file = os.path.join(pkg_share, 'config', 'object_segmentation.yaml')

    depth_topic = LaunchConfiguration('depth_topic')
    mask_topic = LaunchConfiguration('mask_topic')
    detections_topic = LaunchConfiguration('detections_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/realsense/realsense2_camera/depth/image_rect_raw',
            description='Depth image topic to segment.',
        ),
        DeclareLaunchArgument(
            'mask_topic',
            default_value='/perception/segmentation_mask',
            description='Output topic for the segmentation mask.',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/perception/detections',
            description='Output topic for Detection2DArray messages.',
        ),
        Node(
            package='somanet',
            executable='object_segmentation_node',
            name='object_segmentation_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'depth_topic': depth_topic,
                    'mask_topic': mask_topic,
                    'detections_topic': detections_topic,
                },
            ],
        ),
    ])


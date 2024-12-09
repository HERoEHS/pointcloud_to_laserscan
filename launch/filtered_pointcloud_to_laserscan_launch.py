from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='filtered_pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),  # 입력 토픽 수정
                ('scan', '/scan')  # 출력 토픽 수정
            ],
            parameters=[{
                # 'target_frame': 'zed_camera_link',  # 프레임 ID 수정
                'target_frame': '', 
                'transform_tolerance': 1.5,
                'min_height': -0.4,
                'max_height': 0.0,
                'angle_min': -1.57,  # -M_PI/2 1.0472
                'angle_max': 1.57,  # M_PI/2
                'angle_increment': 0.02,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.7,         # 원래 값 0.1
                'range_max': 3.0,          # 7.0
                'use_inf': True,
                'inf_epsilon': 1.0,
                'min_noise_distance': 0.5,          # 원래 1.0
                # 'queue_size': 10000  # 큐 크기 증가
            }],
            name='pointcloud_to_laserscan'
        )
    ])

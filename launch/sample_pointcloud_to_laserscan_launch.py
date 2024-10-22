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
        # Node(
        #     package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
        #     remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
        #     parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 5.0, 'cloud_size': 3000}],
        #     name='cloud_publisher'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         # '--x', '0', '--y', '0', '--z', '0',
        #         # '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'map', '--child-frame-id', 'odom'
        #     ]
        # ),
        # # odom -> zed_camera_link 변환 퍼블리셔 추가
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_odom_to_zed_camera_link',
        #     arguments=[
        #         # '--x', '0', '--y', '0', '--z', '0',
        #         # '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'odom', '--child-frame-id', 'zed_camera_link'
        #     ]
        # ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
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
                'range_max': 2.5,          # 7.0
                'use_inf': True,
                'inf_epsilon': 1.0,
                'min_noise_distance': 0.5,          # 원래 1.0
                # 'queue_size': 10000  # 큐 크기 증가
            }],
            name='pointcloud_to_laserscan'
        )
    ])

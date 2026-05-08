from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():

    package_path = get_package_share_path('imu_ros2_device')
    default_rviz_config_path = package_path / 'rviz/ybimu.rviz'
    print("config path:", default_rviz_config_path)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver_multi',
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('imu_ros2_device'),
        'config',
        'imu_filter_param.yaml'
    )

    imu_filter_node0= Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        remappings=[('imu/data_raw','/imu0/data_raw'),
                    ('imu/mag','/imu0/mag'),
                    ('/imu/data','/imu0/data'),]
        
        
    )

    imu_filter_node1= Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        remappings=[('imu/data_raw','/imu1/data_raw'),
                    ('imu/mag','/imu1/mag'),
                    ('/imu/data','/imu1/data'),]
        

    )

    imu_filter_node2= Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        remappings=[('imu/data_raw','/imu2/data_raw'),
                    ('imu/mag','/imu2/mag'),
                    ('/imu/data','/imu2/data'),]
    
    )

    # filter_node = Node(
    #     package='imu_ros2_device',
    #     executable='imu_filter',
    # )

    imu_muti_tf_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver_tf',
    )

    return LaunchDescription([
        # rviz_arg,
        # rviz_node,
        device_node,
        imu_filter_node0,
        imu_filter_node1,
        imu_filter_node2,

        # filter_node,
        imu_muti_tf_node,
    ])


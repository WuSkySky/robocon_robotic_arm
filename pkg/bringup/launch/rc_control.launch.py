from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    ld = LaunchDescription()

    # 1. 执行查找 CAN 端口的脚本
    find_can_cmd = ExecuteProcess(
        cmd=['bash', './pkg/piper/find_all_can_port.sh'],
        name='find_can_port',
        output='screen'
    )
    ld.add_action(find_can_cmd)

    # 2. 执行 CAN 波特率配置脚本
    can_activate_cmd = ExecuteProcess(
        cmd=['bash', './pkg/piper/can_activate.sh', 'can0', '1000000'],
        name='can_activate',
        output='screen'
    )
    ld.add_action(can_activate_cmd)

    #加入robot_state_publisher_node
    urdf_tutorial_path = get_package_share_path('piper_description')
    default_model_path = urdf_tutorial_path / 'urdf/piper_description.xacro'
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                    description='Absolute path to robot urdf file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        remappings=[
            # 输入重映射
            ('joint_states', '/joint_states_feedback'),  # 从自定义话题接收关节状态
        ],
        parameters=[{'robot_description': robot_description}]
    )
    ld.add_action(model_arg)
    ld.add_action(robot_state_publisher_node)

    #启动piper launch不带rviz文件
    piper_launch_file = os.path.join(
        get_package_share_directory('piper'), 
        'launch',                      
        'start_single_piper.launch.py'                
    )

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_launch_file)
    )

    ld.add_action(piper_launch)


    #启动串口驱动launch文件
    serial_launch_file = os.path.join(
        get_package_share_directory('serial_driver'), 
        'launch',                      
        'serial_driver_bridge_node.launch.py'                
    )

    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(serial_launch_file)
    )

    ld.add_action(serial_launch)

    # 启动IMU的launch文件
    imu_launch_file = os.path.join(
        get_package_share_directory('imu_ros2_device'), 
        'launch',                      
        'ybimu_display.launch.py'                
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file)
    )

    ld.add_action(imu_launch)

    #启动解包串口数据
    unload_serial_data_node = Node(
        package='process_serial_data',
        executable='unload_serial_data',
        name='unload_serial_data_node',
        output='screen'
    )
    ld.add_action(unload_serial_data_node)

    #启动发布目标位姿
    publish_target_pose_node = Node(
        package='process_serial_data',
        executable='pub_target_pose',
        name='publish_target_pose_node',
        output='screen'
    )
    ld.add_action(publish_target_pose_node)

    #启动广播tf
    broadcast_tf_node = Node(
        package='process_serial_data',
        executable='tf_broadcast',
        name='broadcast_tf_node',
        output='screen',
        remappings=[
            ('/imu/data', '/imu/data/filtered')
        ]
    )
    ld.add_action(broadcast_tf_node)

    #启动变换后的tf
    broadcast_offset_tf_node = Node(
        package='robotic_arm_control',
        executable='offset_tf_broadcast',
        name='broadcast_offset_tf_node',
        output='screen'
    )
    ld.add_action(broadcast_offset_tf_node)


    #启动遥控器控制
    rc_control_node = Node(
        package='robotic_arm_control',
        executable='rc_control',
        name='rc_control_node',     
        output='screen'
    )
    ld.add_action(rc_control_node)

    return ld
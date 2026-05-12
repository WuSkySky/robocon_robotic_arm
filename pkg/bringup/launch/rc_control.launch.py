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

    # CAN 波特率配置
    can_activate_cmd = ExecuteProcess(
        cmd=['bash', './pkg/piper/can_activate.sh', 'can0', '1000000'],
        name='can_activate',
        output='screen'
    )
    ld.add_action(can_activate_cmd)

    """
    接受机器人状态发布tf
    sub topic: 
        /joint_states_feedback(sensor_msgs/msg/JointState) 机器人关节状态反馈
    pub topic: 
        /tf(tf2_msgs/msg/TFMessage) 机器人状态的tf变换信息
        /tf_static(tf2_msgs/msg/TFMessage) 机器人状态的tf变换信息
    """
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

    """
    启动piper launch, 该launch提供对piper的控制接口
    sub topic: 
        /enable_flag: std_msgs/msg/Bool)
        /joint_states: sensor_msgs/msg/JointState) 控制机械臂的关节电机位置,速度或力控制
        /pos_cmd: piper_msgs/msg/PosCmd 控制机械臂运动到某个位姿 结算在piper内部
    pub topic: 
        /arm_status: piper_msgs/msg/PiperStatusMsg
        /end_pose: geometry_msgs/msg/Pose
        /end_pose_stamped: geometry_msgs/msg/PoseStamped
        /joint_ctrl: sensor_msgs/msg/JointState
        /joint_states_feedback: sensor_msgs/msg/JointState 机械臂关节状态反馈
        /joint_states_single: sensor_msgs/msg/JointState 机械臂关节状态反馈(和上面一样)
        /parameter_events: rcl_interfaces/msg/ParameterEvent
        /rosout: rcl_interfaces/msg/Log
    """
    piper_launch_file = os.path.join(
        get_package_share_directory('piper'), 
        'launch',                      
        'start_single_piper.launch.py'                
    )

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_launch_file)
    )

    ld.add_action(piper_launch)

    """
    启动IMU驱动的launch文件,接受IMU数据
    pub topic:
        /baro
        /euler
        /imu/data
        /imu/data/filtered
        /imu/data_raw
        /imu/mag
    """
    imu_launch_file = os.path.join(
        get_package_share_directory('imu_ros2_device'), 
        'launch',                      
        'single_imu.launch.py'                
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file)
    )

    ld.add_action(imu_launch)


    # 串口驱动launch文件 接受来自下位机的串口数据，包含摇杆，拨轮和开关的数据
    serial_launch_file = os.path.join(
        get_package_share_directory('serial_driver'), 
        'launch',                      
        'serial_driver_bridge_node.launch.py'                
    )

    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(serial_launch_file)
    )

    ld.add_action(serial_launch)

    # 启动串口解包，将串口原始字节数据解包成ROS消息
    unload_serial_data_node = Node(
        package='process_serial_data',
        executable='unload_serial_data',
        name='unload_serial_data_node',
        output='screen'
    )
    ld.add_action(unload_serial_data_node)

    # 发布目标位姿，根据遥控器输入和IMU数据计算目标位姿，并发布ROS话题
    publish_target_pose_node = Node(
        package='process_serial_data',
        executable='pub_target_pose',
        name='publish_target_pose_node',
        output='screen',
        remappings=[
            ('/imu/data', '/imu/data/filtered')
        ]
    )
    ld.add_action(publish_target_pose_node)

    # 启动目标位姿tf广播，将目标位姿以tf的形式广播出来，供RViz2可视化和控制处理使用
    broadcast_tf_node = Node(
        package='process_serial_data',
        executable='tf_broadcast',
        name='broadcast_tf_node',
        output='screen',
    )
    ld.add_action(broadcast_tf_node)

    # 启动发送夹爪控制命令的节点
    pub_gripper_control_node = Node(
        package='process_serial_data',
        executable='pub_gripper_control',
        name='pub_gripper_control_node',
        output='screen'
    )
    ld.add_action(pub_gripper_control_node)

    # 启动发布纠正tf，将初始目标位姿与机械臂球腕中心对齐
    broadcast_offset_tf_node = Node(
        package='robotic_arm_control',
        executable='offset_tf_broadcast',
        name='broadcast_offset_tf_node',
        output='screen'
    )
    ld.add_action(broadcast_offset_tf_node)

    # 启动遥控器控制 使用目标位姿控制机械臂运动
    rc_control_node = Node(
        package='robotic_arm_control',
        executable='rc_control',
        name='rc_control_node',     
        output='screen'
    )
    ld.add_action(rc_control_node)

    return ld
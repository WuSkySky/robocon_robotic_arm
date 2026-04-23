import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_matrix
import numpy as np
import tf2_ros
import roboticstoolbox as rtb
import numpy as np
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory
import os
from filter.filter import SecondOrderButterworthLowPass

class RcControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # pub
        self.joint_angle_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # sub
        # link1到link6的关节角度反馈
        self.joint_angle_subscriber = self.create_subscription(
            JointState,
            '/joint_states_single',
            self.joint_angle_subscriber_callback,
            10
        )

        # 来自下位机器的夹爪控制命令
        self.gripper_control_subscriber = self.create_subscription(
            Float64,
            '/gripper_control',
            self.gripper_control_subscriber_callback,
            10
        )

        # timer
        self.arm_control_timer = self.create_timer(0.016, self.arm_control_timer_callback)
        
        # flag
        self.first_pose = False

        # slover
        piper_description_path = get_package_share_directory('piper_description')
        robot_urdf_file_path = os.path.join(piper_description_path, 'urdf', 'piper_no_gripper_description.urdf')
        self.arm_slover = ArmSlover(robot_urdf_file_path) 
        self.get_pose_msg = JointState()

        # filter
        self.solver_output_filter = SecondOrderButterworthLowPass(Wn=0.25)

    # 回调函数
    def joint_angle_subscriber_callback(self, msg):
        self.get_pose_msg.position = msg.position
    
    def gripper_control_subscriber_callback(self, msg):
        self.gripper_control_angle = msg.data

    def arm_control_timer_callback(self):
        try:
            tf_base_link_to_target_pose_matched = self.tf_buffer.lookup_transform(
                source_frame='target_pose_matched',
                target_frame='base_link',
                time=rclpy.time.Time()
            )
            tf_target_pose_matched_to_final_pose = self.tf_buffer.lookup_transform(
                source_frame='link6',
                target_frame='base_link',
                time=rclpy.time.Time()
            )

            _ = self.gripper_control_angle
        except Exception as e:
            self.get_logger().warn(f"rc_control_node 获取目标 tf 或夹爪控制命令失败,等待上游数据...")
            return

        target_matrix_pose_matched = self.transform_to_4x4(tf_base_link_to_target_pose_matched.transform)
        target_matrix_final = self.transform_to_4x4(tf_target_pose_matched_to_final_pose.transform)
    
        control_joint_towards, success_flag_towards = self.arm_slover.ik(
            target_matrix_final, 
            q0=self.get_pose_msg.position[3:6] if self.get_pose_msg.position[3:6] is not None else np.zeros(3),
            start=4,  # 从当前关节角度开始解算
            end=6,
            mask=[0, 0, 0, 1, 1, 1]  # 只考虑末端执行器的角度
            )
        control_joint_base, success_flag_angles = self.arm_slover.ik(
            target_matrix_pose_matched, 
            q0=self.get_pose_msg.position[0:3] if self.get_pose_msg.position[0:3] is not None else np.zeros(3),
            start=1,  # 从当前关节角度开始解算
            end=3,
            mask=[1, 1, 1, 0, 0, 0]  # 只考虑基座的姿态
            )
        control_joint_angles = control_joint_base[0:3].tolist() + control_joint_towards[3:6].tolist()

        if success_flag_towards and success_flag_angles:
            control_msg = JointState()

            control_msg.header.stamp = self.get_clock().now().to_msg()
            control_msg.header.frame_id = ''
            
            control_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            control_msg.position =  self.solver_output_filter.filter_multiple(list(control_joint_angles)+[self.gripper_control_angle])

            self.get_logger().info(f"IK 解算成功, 关节角度: {control_msg.position}")
            self.joint_angle_publisher.publish(control_msg)
        else:
            self.get_logger().warn(f"IK 解算失败")

    # 工具函数
    def transform_to_4x4(self, transform):
        """
        将 geometry_msgs/Transform 转换为 4x4 矩阵
        """
        
        # 提取四元数
        q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ]
        
        # 获取旋转矩阵
        R = quaternion_matrix(q)
        
        # 设置平移
        R[0, 3] = transform.translation.x
        R[1, 3] = transform.translation.y
        R[2, 3] = transform.translation.z
        
        return R

class ArmSlover:
    def __init__(self,urdf_path):
        """
        Args:
            urdf_path: str, URDF文件路径
        """
        self.robot = rtb.ERobot.URDF(urdf_path)

    def ik(self, target, q0=np.zeros(6),start=None, end=None, mask=[]):
        """
        逆运动学解算得到关节角度
        Args:
            target: matrix-like 4x4
            q0: array-like, 初始关节角度
        Returns:
            thetas: array-like, 关节角度
            success: bool, 是否成功求解, 成功1, 失败0
        """

        result = self.robot.ikine_LM(target,q0=q0,start=self.robot.links[start],end=self.robot.links[end],mask=mask)
        return result[0], result[1]
def main(args=None):
    rclpy.init(args=args)
    node = RcControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_matrix
import numpy as np
import tf2_ros
import roboticstoolbox as rtb
import numpy as np
import numpy as np
from sensor_msgs.msg import JointState
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

        # timer
        self.arm_control_timer = self.create_timer(0.016, self.arm_control_timer_callback)
        self.first_pose = True

        # slover
        piper_description_path = get_package_share_directory('piper_description')
        robot_urdf_file_path = os.path.join(piper_description_path, 'urdf', 'piper_no_gripper_description.urdf')
        self.arm_slover = ArmSlover(robot_urdf_file_path) 

        # filter
        self.solver_output_filter = SecondOrderButterworthLowPass(Wn=0.25)

    # 回调函数
    def arm_control_timer_callback(self):
        try:
            tf_base_link_to_target_pose_matched = self.tf_buffer.lookup_transform(
                source_frame='target_pose_matched',
                target_frame='base_link',
                time=rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"rc_control获取tf失败,等待tf中...")
            return

        target_matrix = self.transform_to_4x4(tf_base_link_to_target_pose_matched.transform)

        control_joint_angles, success_flag = self.arm_slover.ik(target_matrix)

        if success_flag:
            control_msg = JointState()

            control_msg.header.stamp = self.get_clock().now().to_msg()
            control_msg.header.frame_id = ''
            
            control_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            control_msg.position =  self.solver_output_filter.filter_multiple(control_joint_angles)

            self.get_logger().info(f"IK 解算成功, 关节角度: {control_msg.position}")
            self.joint_angle_publisher.publish(control_msg)
        else:
            self.get_logger().warn(f"IK 解算失败")

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

    def ik(self, target):
        """
        逆运动学解算得到关节角度
        Args:
            target: matrix-like 4x4
        Returns:
            thetas: array-like, 关节角度
            success: bool, 是否成功求解, 成功1, 失败0
        """
        result = self.robot.ik_LM(target,q0=np.zeros(6))
        return result[0], result[1]

def main(args=None):
    rclpy.init(args=args)
    node = RcControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


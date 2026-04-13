import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import TransformStamped
import numpy as np
import tf2_ros
import tf_transformations

class RcControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')

        self.tf_base_link_rc_base=None

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # pub
        self.pose_publisher = self.create_publisher(
            Pose,
            '/pose_ctrl',
            10
        )

        # timer
        self.recode_fix_tf_timer = self.create_timer(3, self.recode_fix_tf_timer_callback)
        self.pub_tf_base_link_to_rc_base_timer = self.create_timer(0.01, self.pub_tf_base_link_to_rc_base_timer_callback)
        self.arm_control_timer = self.create_timer(1, self.arm_control_timer_callback)
        

    # 回调函数
    def recode_fix_tf_timer_callback(self):
        if self.tf_base_link_rc_base is None:
            self.tf_base_link_rc_base = self.calculate_tf_pre_fix(
                self.tf_buffer.lookup_transform(
                    source_frame='target_pose',
                    target_frame='rc_base',
                    time=rclpy.time.Time()
                ),
                self.tf_buffer.lookup_transform(
                    source_frame='link6',
                    target_frame='base_link',
                    time=rclpy.time.Time()
                ),
                'base_link'
            )

    def pub_tf_base_link_to_rc_base_timer_callback(self):
        if self.tf_base_link_rc_base is not None:
            self.tf_base_link_rc_base.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.tf_base_link_rc_base)
            

    def arm_control_timer_callback(self):
        pass


    def calculate_tf_pre_fix(self, tf_n, tf_r, tf_m_parent_frame):
        """
        计算tf_m, 使得 tf_m @ tf_n = tf_r
        
        Args:
            tf_n: 被修正的tf
            tf_r: 参考tf
        """
        # 提取n的平移和旋转
        n_trans = tf_n.transform.translation
        n_rot = tf_n.transform.rotation
        
        # 提取r的平移和旋转
        r_trans = tf_r.transform.translation
        r_rot = tf_r.transform.rotation
        
        # 转换为变换矩阵
        # n的变换矩阵
        n_matrix = tf_transformations.quaternion_matrix([
            n_rot.x, n_rot.y, n_rot.z, n_rot.w
        ])
        n_matrix[0][3] = n_trans.x
        n_matrix[1][3] = n_trans.y
        n_matrix[2][3] = n_trans.z
        
        # r的变换矩阵
        r_matrix = tf_transformations.quaternion_matrix([
            r_rot.x, r_rot.y, r_rot.z, r_rot.w
        ])
        r_matrix[0][3] = r_trans.x
        r_matrix[1][3] = r_trans.y
        r_matrix[2][3] = r_trans.z
        
        # 计算m的变换矩阵: m = r * n⁻¹
        n_inv_matrix = np.linalg.inv(n_matrix)
        m_matrix = r_matrix @ n_inv_matrix
        
        # 从矩阵中提取平移和旋转
        m_translation = m_matrix[0:3, 3]
        m_rotation = tf_transformations.quaternion_from_matrix(m_matrix)
        
        # 创建tf_m
        tf_m = TransformStamped()
        tf_m.header.frame_id = tf_m_parent_frame
        tf_m.child_frame_id = tf_n.header.frame_id        
        tf_m.transform.translation.x = float(m_translation[0])
        tf_m.transform.translation.y = float(m_translation[1])
        tf_m.transform.translation.z = float(m_translation[2])
        
        tf_m.transform.rotation.x = float(m_rotation[0])
        tf_m.transform.rotation.y = float(m_rotation[1])
        tf_m.transform.rotation.z = float(m_rotation[2])
        tf_m.transform.rotation.w = float(m_rotation[3])

        return tf_m

    def calculate_tf_post_fix(self, tf_m, tf_r, tf_n_child_frame):
        """
        计算tf_n, 使得 tf_m @ tf_n = tf_r
        
        Args:
            tf_m: 被修正的tf
            tf_r: 参考tf
        """
        # 提取m的平移和旋转
        m_trans = tf_m.transform.translation
        m_rot = tf_m.transform.rotation
        
        # 提取r的平移和旋转
        r_trans = tf_r.transform.translation
        r_rot = tf_r.transform.rotation
        
        # 转换为变换矩阵
        # m的变换矩阵
        m_matrix = tf_transformations.quaternion_matrix([
            m_rot.x, m_rot.y, m_rot.z, m_rot.w
        ])
        m_matrix[0][3] = m_trans.x
        m_matrix[1][3] = m_trans.y
        m_matrix[2][3] = m_trans.z
        
        # r的变换矩阵
        r_matrix = tf_transformations.quaternion_matrix([
            r_rot.x, r_rot.y, r_rot.z, r_rot.w
        ])
        r_matrix[0][3] = r_trans.x
        r_matrix[1][3] = r_trans.y
        r_matrix[2][3] = r_trans.z
        
        # 计算n的变换矩阵: n = m⁻¹ * r
        m_inv_matrix = np.linalg.inv(m_matrix)
        n_matrix = m_inv_matrix @ r_matrix
        
        # 从矩阵中提取平移和旋转
        n_translation = n_matrix[0:3, 3]
        n_rotation = tf_transformations.quaternion_from_matrix(n_matrix)
        
        # 创建tf_n
        tf_n = TransformStamped()
        tf_n.header.frame_id = tf_m.child_frame_id
        tf_n.child_frame_id = tf_n_child_frame      
        tf_n.transform.translation.x = float(n_translation[0])
        tf_n.transform.translation.y = float(n_translation[1])
        tf_n.transform.translation.z = float(n_translation[2])
        
        tf_n.transform.rotation.x = float(n_rotation[0])
        tf_n.transform.rotation.y = float(n_rotation[1])
        tf_n.transform.rotation.z = float(n_rotation[2])
        tf_n.transform.rotation.w = float(n_rotation[3])

        return tf_n

            
def main(args=None):
    rclpy.init(args=args)
    # node = RcControlNode()
    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


import math
import time
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
import tf_transformations

class OffsetTfBroadcastNode(Node):
    def __init__(self):
        super().__init__('offset_tf_broadcast_node')

        # flag
        self.init_pose_used = False
        self.pre_offset_used = False
        self.post_offset_used = False

        # pub
        self.init_joint_angle_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # static tf broadcaster
        self.static_pre_offset_tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_post_offset_tf_broadcaster = StaticTransformBroadcaster(self)

        # timer
        self.init_pose_single_use_timer = self.create_timer(3, self.init_pose_single_use_timer_callback)
        self.pre_offset_single_use_timer = self.create_timer(1, self.pre_offset_single_use_timer_callback)
        self.post_offset_single_use_timer = self.create_timer(1, self.post_offset_single_use_timer_callback)
        

    # 回调函数
    def init_pose_single_use_timer_callback(self):
        if self.init_pose_used:
            return
        
        control_msg = JointState()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = ''
        control_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        control_msg.position = [0.0, 0.7, -1.04, -0.2, 0.2, 0.3]

        self.init_joint_angle_publisher.publish(control_msg)

        self.get_logger().info(f"发布init pose")

        self.init_pose_used = True
        

    def pre_offset_single_use_timer_callback(self):
        if self.pre_offset_used or not self.init_pose_used:
            return
        
        try:
            tf_rc_odom_to_target_pose = self.tf_buffer.lookup_transform(
                source_frame='target_pose',
                target_frame='rc_odom',
                time=rclpy.time.Time()
            )

            tf_base_link_to_link6 = self.tf_buffer.lookup_transform(
                source_frame='link6',
                target_frame='base_link',
                time=rclpy.time.Time()
            )
        except:
            return

        tf_base_link_to_link6_quat = [tf_base_link_to_link6.transform.rotation.x,
             tf_base_link_to_link6.transform.rotation.y,
             tf_base_link_to_link6.transform.rotation.z,
             tf_base_link_to_link6.transform.rotation.w]

        rotation_y_minus_90 = quaternion_from_euler(0.0, -math.pi/2, 0.0)
        new_quat = quaternion_multiply(tf_base_link_to_link6_quat, rotation_y_minus_90)

        tf_base_link_to_link6_offset = TransformStamped()
        tf_base_link_to_link6_offset.header.frame_id = 'base_link'
        tf_base_link_to_link6_offset.child_frame_id = '_'
        tf_base_link_to_link6_offset.transform.translation = tf_base_link_to_link6.transform.translation
        tf_base_link_to_link6_offset.transform.rotation.x = new_quat[0]
        tf_base_link_to_link6_offset.transform.rotation.y = new_quat[1]
        tf_base_link_to_link6_offset.transform.rotation.z = new_quat[2]
        tf_base_link_to_link6_offset.transform.rotation.w = new_quat[3]
        
        tf_base_link_to_rc_odom = self.calculate_pre_offset_tf(
            tf_rc_odom_to_target_pose,
            tf_base_link_to_link6_offset
        )
        tf_base_link_to_rc_odom.child_frame_id = 'rc_odom'
        tf_base_link_to_rc_odom.header.stamp = self.get_clock().now().to_msg()

        self.static_pre_offset_tf_broadcaster.sendTransform(tf_base_link_to_rc_odom)

        self.get_logger().info(f"发布pre-offset tf")

        self.pre_offset_used = True

    def post_offset_single_use_timer_callback(self):
        if self.post_offset_used or not self.init_pose_used:
            return

        try:
            # 保证pre-offset tf已经发布
            self.tf_buffer.lookup_transform(
                source_frame='rc_odom',
                target_frame='base_link',
                time=rclpy.time.Time()
            )
            
            tf_base_link_to_target_pose = self.tf_buffer.lookup_transform(
                source_frame='target_pose',
                target_frame='base_link',
                time=rclpy.time.Time()
            )

            tf_base_link_to_link6 = self.tf_buffer.lookup_transform(
                source_frame='link6',
                target_frame='base_link',
                time=rclpy.time.Time()
            )
        except:
            return

        tf_target_pose_to_target_pose_matched = self.calculate_post_offset_tf(
            tf_base_link_to_target_pose,
            tf_base_link_to_link6
        )
        tf_target_pose_to_target_pose_matched.child_frame_id = 'target_pose_matched'
        tf_target_pose_to_target_pose_matched.header.stamp = self.get_clock().now().to_msg()

        self.static_post_offset_tf_broadcaster.sendTransform(tf_target_pose_to_target_pose_matched)

        self.get_logger().info(f"发布post-offset tf")

        self.post_offset_used = True

    def calculate_pre_offset_tf(self, tf_n, tf_r):
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
        tf_m.header.frame_id = tf_r.header.frame_id
        tf_m.child_frame_id = tf_n.child_frame_id
        tf_m.transform.translation.x = float(m_translation[0])
        tf_m.transform.translation.y = float(m_translation[1])
        tf_m.transform.translation.z = float(m_translation[2])
        
        tf_m.transform.rotation.x = float(m_rotation[0])
        tf_m.transform.rotation.y = float(m_rotation[1])
        tf_m.transform.rotation.z = float(m_rotation[2])
        tf_m.transform.rotation.w = float(m_rotation[3])

        return tf_m

    def calculate_post_offset_tf(self, tf_m, tf_r):
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
        tf_n.child_frame_id = tf_r.child_frame_id      
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
    node = OffsetTfBroadcastNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


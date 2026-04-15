import time
import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd,PiperStatusMsg
from tf2_ros import TransformStamped
import numpy as np
import tf2_ros
import math
import tf_transformations

class RcControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # pub
        self.pose_publisher = self.create_publisher(
            PiperStatusMsg,
            '/joint_states',
            10
        )

        # timer
        self.arm_control_timer = self.create_timer(0.01, self.arm_control_timer_callback)
        self.first_pose = True
    # 获取四元数
    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
    
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    # 回调函数
    def arm_control_timer_callback(self):
        try:
            tf_base_link_to_link6 = self.tf_buffer.lookup_transform(
                source_frame='target_pose_matched',
                target_frame='base_link',
                time=rclpy.time.Time()
            )

            #生成PiperStatusMsg消息
            PiperStatusMsg_msg = PiperStatusMsg()

            if self.first_pose == True:
                self.pose_publisher.publish(PiperStatusMsg_msg)
                self.first_pose = False
                self.get_logger().info(f"SetFirstPose Really")

            PiperStatusMsg_msg.ctrl_mode = 0x01
            PiperStatusMsg_msg.arm_status = 0x00
            PiperStatusMsg_msg.mode_feedback = 0x00
            PiperStatusMsg_msg.teach_status = 0x00
            PiperStatusMsg_msg.motion_status = 0x00
            PiperStatusMsg_msg.trajectory_num = 0x00
            PiperStatusMsg_msg.err_code = 0x00
            PiperStatusMsg_msg.joint_1_angle_limit = False
            PiperStatusMsg_msg.joint_2_angle_limit = False
            PiperStatusMsg_msg.joint_3_angle_limit = False
            PiperStatusMsg_msg.joint_4_angle_limit = False
            PiperStatusMsg_msg.joint_5_angle_limit = False
            PiperStatusMsg_msg.joint_6_angle_limit = False
            PiperStatusMsg_msg.communication_status_joint_1 = False
            PiperStatusMsg_msg.communication_status_joint_2 = False
            PiperStatusMsg_msg.communication_status_joint_3 = False
            PiperStatusMsg_msg.communication_status_joint_4 = False
            PiperStatusMsg_msg.communication_status_joint_5 = False
            PiperStatusMsg_msg.communication_status_joint_6 = False


            self.get_logger().info(f"Publishing PiperStatusMsg: ctrl_mode={PiperStatusMsg_msg.ctrl_mode}")
            self.pose_publisher.publish(PiperStatusMsg_msg)
        except Exception as e:
            self.get_logger().error(f"Error in arm_control_timer_callback: {e}")

        


      
def main(args=None):
    rclpy.init(args=args)
    node = RcControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


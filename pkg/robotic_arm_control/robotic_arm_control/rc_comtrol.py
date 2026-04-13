import time
import rclpy
from rclpy.node import Node
from piper_msgs.msg import PosCmd
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
            PosCmd,
            '/pos_cmd',
            10
        )

        # timer
        self.arm_control_timer = self.create_timer(0.01, self.arm_control_timer_callback)
        
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
            #生成PosCmd消息
            PosCmd_msg = PosCmd()

            qx = tf_base_link_to_link6.transform.rotation.x
            qy = tf_base_link_to_link6.transform.rotation.y
            qz = tf_base_link_to_link6.transform.rotation.z
            qw = tf_base_link_to_link6.transform.rotation.w

            # 转换为欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

            #PosCmd里面的x,y,z填充
            PosCmd_msg.x = tf_base_link_to_link6.transform.translation.x
            PosCmd_msg.y = tf_base_link_to_link6.transform.translation.y
            PosCmd_msg.z = tf_base_link_to_link6.transform.translation.z
            #PosCmd里面的roll,pitch,yaw填充
            PosCmd_msg.roll = roll
            PosCmd_msg.pitch = pitch
            PosCmd_msg.yaw = yaw

            self.get_logger().info(f"Publishing PosCmd: x={PosCmd_msg.x}, y={PosCmd_msg.y}, z={PosCmd_msg.z}, roll={PosCmd_msg.roll}, pitch={PosCmd_msg.pitch}, yaw={PosCmd_msg.yaw}")
            self.pose_publisher.publish(PosCmd_msg)
        except Exception as e:
            self.get_logger().error(f"Error in arm_control_timer_callback: {e}")

        


      
def main(args=None):
    rclpy.init(args=args)
    node = RcControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


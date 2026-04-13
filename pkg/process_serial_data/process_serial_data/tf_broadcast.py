import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,TransformStamped           # 用于发布处理后的tf位姿数据
import tf2_ros

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')

        self.parent_frame = 'rc_odom'      # 父坐标系
        self.child_frame = 'target_pose' # 子坐标系

        # --- 1. 创建订阅者：接收原始数据 ---
        self.subscription = self.create_subscription(
            Pose,                      # 消息类型
            '/target_pose',                # 话题名
            self.pose_data_received_callback,      # 回调函数
            10                           # 队列长度
        )

        # --- 2. 创建发布者：发送tf的数据 ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def pose_data_received_callback(self, msg: Pose):
        """
        收到 Pose 消息发送tf的回调函数。
        """
        t = TransformStamped()

        # 填充时间戳和坐标系 ID
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # 平移部分：直接使用 Pose 中的 position
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        # 旋转部分：直接使用 Pose 中的 orientation（四元数）
        t.transform.rotation = msg.orientation

        # 广播变换
        self.tf_broadcaster.sendTransform(t)
        
def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)  # 保持节点运行，等待回调函数被调用
    node.destroy_node()
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String          # 用于接收原始数据
from geometry_msgs.msg import Pose,TransformStamped           # 用于发布处理后的tf位姿数据
from my_msgs.msg import PosCmd        # 假设这是你自定义的位姿消息
import tf2_ros

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')

        # --- 1. 创建订阅者：接收原始数据 ---
        self.subscription = self.create_subscription(
            Pose,                      # 消息类型
            '/target_pose',                # 话题名
            self.tf_data_callback,      # 回调函数
            10                           # 队列长度
        )

        # --- 2. 创建发布者：发送tf的数据 ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.parent_frame = 'base_link'      # 父坐标系
        self.child_frame = 'target_link' # 子坐标系

        # --- 3. 创建发布者：发送PosCmd的数据 ---
        self.publisher = self.create_publisher(
            PosCmd,                     # 消息类型
            '/processed_pose',           # 话题名processed_serial_pose
            10                           # 队列长度
        )
    def tf_data_callback(self, msg: Pose):
        """
        收到 Pose 消息发送PosCmd和tf时的回调函数。
        """
        # PosCmd_msg = PosCmd()

        # PosCmd_msg.x = msg.position.x
        # PosCmd_msg.y = msg.position.y
        # PosCmd_msg.position.z = msg.position.z
        # PosCmd_msg.orientation.w = msg.orientation.w
        # PosCmd_msg.orientation.x = msg.orientation.x
        # PosCmd_msg.orientation.y = msg.orientation.y
        # PosCmd_msg.orientation.z = msg.orientation.z

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

        # 发布处理后的消息
        self.tf_broadcaster.sendTransform(t)
        # self.publisher.publish(PosCmd_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)  # 保持节点运行，等待回调函数被调用
    node.destroy_node()
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformStamped
from sensor_msgs.msg import Imu 
from example_interfaces.srv import SetBool
import tf2_ros

class ImuMutiBroadcastNode(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcast_node')
        # 参数
        self.tf_enabled = False

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tf_broadcaster_odom = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odom_to_base) 

        # 发布静态变换
        self.static_imu0_base_tf_broadcaster = StaticTransformBroadcaster(self)

        # 接收Imu数据的订阅者
        self.subscription = self.create_subscription(
            Imu,                      # 消息类型
            '/imu0/data/aligned',                # 话题名
            self.imu0_data_received_callback,      # 回调函数
            10                           # 队列长度
        )

        self.subscription = self.create_subscription(
            Imu,                      # 消息类型
            '/imu1/data/aligned',                # 话题名
            self.imu1_data_received_callback,      # 回调函数
            10                           # 队列长度
        )
        self.subscription = self.create_subscription(
            Imu,                      # 消息类型
            '/imu2/data/aligned',                # 话题名
            self.imu2_data_received_callback,      # 回调函数
            10                           # 队列长度
        )
        self.tf_broadcaster_imu0 = tf2_ros.TransformBroadcaster(self)
        self.tf_broadcaster_imu1 = tf2_ros.TransformBroadcaster(self)
        self.tf_broadcaster_imu2 = tf2_ros.TransformBroadcaster(self)

        # 服务名2：第二个用户处理近似纠正的情况，开启或关闭 TF 发布
        self.srv = self.create_service(SetBool, '/align_user', self.callback)
        self.get_logger().info('对齐服务已启动，等待 ros2 service call ...')

    def callback(self, request, response):
        self.tf_enabled = request.data
        if self.tf_enabled:
            response.success = True
            response.message = "TF 发布已开启"
            self.get_logger().info("TF 发布已开启")
        else:
            response.success = True
            response.message = "TF 发布已关闭"
            self.get_logger().info("TF 发布已关闭")
        return response

    def publish_odom_to_base(self):
        if not self.tf_enabled:
            return  # 不发布 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'odom_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = 0.7071068
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.7071068
        t.transform.rotation.z = 0.0

        self.tf_broadcaster_odom.sendTransform(t)

    def imu0_data_received_callback(self, msg: Imu):
        """
        收到 /imu0/data_raw 消息发送tf的回调函数。
        """
        # 填充时间戳和坐标系 ID
        # print("test")
        if not self.tf_enabled:
            return  # 不发布
        t0 = TransformStamped()
        t0.header.stamp = self.get_clock().now().to_msg()
        t0.header.frame_id = 'odom_link'
        t0.child_frame_id = 'imu{id}_aligned_link'.format(id=0)

        t0.transform.translation.x = 0.0
        t0.transform.translation.y = 0.0
        t0.transform.translation.z = 0.0

        # 旋转部分：直接使用 orientation（四元数）
        # t0.transform.rotation.w = 0.0
        # t0.transform.rotation.x = -1.0
        # t0.transform.rotation.y = 0.0
        # t0.transform.rotation.z = 0.0
        t0.transform.rotation = msg.orientation

        self.tf_broadcaster_imu0.sendTransform(t0)
        self.static_imu0_base_tf_broadcaster.sendTransform(t0)

    def imu1_data_received_callback(self, msg: Imu):
        """
        收到 /imu1/data_raw 消息发送tf的回调函数。
        """
        if not self.tf_enabled:
            return  # 不发布
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'imu{id}_aligned_link'.format(id=0)
        t1.child_frame_id = 'imu{id}_aligned_link'.format(id=1)

        t1.transform.translation.x = 0.25
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0

        # 旋转部分：直接使用 orientation（四元数）
        # t1.transform.rotation.w = 0.0
        # t1.transform.rotation.x = -1.0
        # t1.transform.rotation.y = 0.0
        # t1.transform.rotation.z = 0.0
        t1.transform.rotation = msg.orientation

        self.tf_broadcaster_imu1.sendTransform(t1)


    def imu2_data_received_callback(self, msg: Imu):
        """
        收到 /imu2/data_raw 消息发送tf的回调函数。
        """
        if not self.tf_enabled:
            return  # 不发布 TF
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'imu{id}_aligned_link'.format(id=1)
        t2.child_frame_id = 'target_pose_matched'

        t2.transform.translation.x = 0.15
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0

        # 旋转部分：直接使用 orientation（四元数）
        # t2.transform.rotation.w = 0.0
        # t2.transform.rotation.x = -1.0
        # t2.transform.rotation.y = 0.0
        # t2.transform.rotation.z = 0.0
        t2.transform.rotation = msg.orientation
        self.tf_broadcaster_imu2.sendTransform(t2)

def main(args=None):
    rclpy.init(args=args)
    node = ImuMutiBroadcastNode()
    rclpy.spin(node)  # 保持节点运行，等待回调函数被调用
    node.destroy_node()
    rclpy.shutdown()
import rclpy
from rclpy.node import Node
from interface.srv import AlignImu  # 用现成接口最简单，实际可以自定义
from sensor_msgs.msg import Imu 
from rclpy.clock import Clock
import numpy as np
from scipy.spatial.transform import Rotation

# 如果想要自定义请求/响应，可以用 std_srvs/srv/Trigger 或自己写 .srv 文件
# 这里演示用 SetBool，把 data(bool) 当作标识，用 message(string) 返回确认
# 但更清晰的是用 example_interfaces/SetString 或者自定义，不过为了零依赖，用 SetBool 演示

class AlignService(Node):
    def __init__(self):
        super().__init__('align_service')
        # 参数定义
        self.recoded_imu_frame = [None, None, None]  # 存储每个 IMU 的参考 frame
        self.imu_publishers_fix = [None, None, None]     # 存储每个 IMU 的orientation矩阵

        # 创建服务：类型、服务名、回调:第一个处理手臂朝下的情况，进行一次初始化
        self.srv = self.create_service(AlignImu, '/align_imu', self.align_callback)
        self.get_logger().info('对齐服务已启动，等待 ros2 service call ...')


        # 订阅 IMU 数据
        self.subscription0 = self.create_subscription(
            Imu,                      # 消息类型
            '/imu0/data/filtered',                # 话题名
            self.imu0_data_received_callback,      # 回调函数             # 已经就绪，直接成功
            10                           # 队列长度
        )

        self.subscription1 = self.create_subscription(
            Imu,                      # 消息类型
            '/imu1/data/filtered',                # 话题名
            self.imu1_data_received_callback,      # 回调函数
            10                           # 队列长度
        )

        self.subscription2 = self.create_subscription(
            Imu,                      # 消息类型
            '/imu2/data/filtered',                # 话题名
            self.imu2_data_received_callback,      # 回调函数
            10                           # 队列长度
        )
        self.aligned_pub_0 = self.create_publisher(Imu, '/imu0/data/aligned', 10)
        self.aligned_pub_1 = self.create_publisher(Imu, '/imu1/data/aligned', 10)
        self.aligned_pub_2 = self.create_publisher(Imu, '/imu2/data/aligned', 10)

    def apply_alignment(self, msg: Imu, recorded_frame:list):
        if recorded_frame is None:
            self.get_logger().warn("waiting for reference frame...")
            return msg  # 没有参考帧，无法对齐，直接返回原始数据
        
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = f"imu_aligned_link"

        q_ref = recorded_frame   # [w, x, y, z]
        q_cur = np.array([msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z])

        # 2. 转为 scipy 的 [x, y, z, w] 顺序
        rot_ref = Rotation.from_quat([q_ref[1], q_ref[2], q_ref[3], q_ref[0]])
        rot_cur = Rotation.from_quat([q_cur[1], q_cur[2], q_cur[3], q_cur[0]])

        # 3. 相对旋转：从当前 B 系转到参考 B0 系  delta_rot = q_ref⁻¹ * q_cur
        rot_delta = (rot_ref.inv() * rot_cur).as_quat()  # 得到 [x, y, z, w] 顺序的四元数

        # # 4. 旋转加速度和角速度
        # acc_raw = np.array([msg.linear_acceleration.x,
        #                     msg.linear_acceleration.y,
        #                     msg.linear_acceleration.z])
        # acc_corr = rot_delta.apply(acc_raw)
        # imu_msg.linear_acceleration.x = acc_corr[0]
        # imu_msg.linear_acceleration.y = acc_corr[1]
        # imu_msg.linear_acceleration.z = acc_corr[2]

        # gyro_raw = np.array([msg.angular_velocity.x,
        #                     msg.angular_velocity.y,
        #                     msg.angular_velocity.z])
        # gyro_corr = rot_delta.apply(gyro_raw)
        # imu_msg.angular_velocity.x = gyro_corr[0]
        # imu_msg.angular_velocity.y = gyro_corr[1]
        # imu_msg.angular_velocity.z = gyro_corr[2]

        imu_msg.orientation.w = rot_delta[3]
        imu_msg.orientation.x = rot_delta[0]
        imu_msg.orientation.y = rot_delta[1]
        imu_msg.orientation.z = rot_delta[2]
        return imu_msg

    def align_callback(self, request, response):
        """每收到一次请求，执行此回调一次"""
        if request.request_align:
            response.success = True
            response.message = "already aligned"
            self.recoded_imu_frame=self.imu_publishers_fix.copy()  # 记录当前的 orientation 作为对齐参考
        else:
            response.success = True
            response.message = "nothing done"
            self.if_align = False
        return response
    
    def imu0_data_received_callback(self, msg: Imu,):
        """
        收到 /imu0/data_raw 消息发送tf的回调函数。
        """
        self.imu_publishers_fix[0] = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])

        imu_msg_align = self.apply_alignment(msg,self.recoded_imu_frame[0])
        self.aligned_pub_0.publish(imu_msg_align)


    def imu1_data_received_callback(self, msg: Imu):
        """
        收到 /imu1/data_raw 消息发送tf的回调函数。
        """
        self.imu_publishers_fix[1] = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])

        imu_msg_align = self.apply_alignment(msg,self.recoded_imu_frame[1])
        self.aligned_pub_1.publish(imu_msg_align)


    def imu2_data_received_callback(self, msg: Imu):
        """
        收到 /imu2/data/filtered 消息发送tf的回调函数。
        """
        self.imu_publishers_fix[2] = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])

        imu_msg_align = self.apply_alignment(msg,self.recoded_imu_frame[2])
        self.aligned_pub_2.publish(imu_msg_align)

def main(args=None):
    rclpy.init(args=args)
    node = AlignService()
    rclpy.spin(node)          # 保持运行，处理请求
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
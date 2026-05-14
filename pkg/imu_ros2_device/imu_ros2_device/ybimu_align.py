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
        self.imu_publishers_fix = [None, None, None]     # 存储每个 IMU 的orientation矩阵
        self.imu_muti_bool = [False, False, False]  # 三个imu是否对齐
        self.last_accel_0 = None
        self.last_accel_1 = None
        self.last_accel_2 = None
        self.if_align = False

        # 创建服务：类型、服务名、回调
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

    def apply_alignment(self, id , msg: Imu):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = f"imu{id}_aligned_link"

        q_ref = self.imu_publishers_fix[id]   # [w, x, y, z]
        q_cur = np.array([msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z])

        # 2. 转为 scipy 的 [x, y, z, w] 顺序
        rot_ref = Rotation.from_quat([q_ref[1], q_ref[2], q_ref[3], q_ref[0]])
        rot_cur = Rotation.from_quat([q_cur[1], q_cur[2], q_cur[3], q_cur[0]])

        # 3. 相对旋转：从当前 B 系转到参考 B0 系  delta_rot = q_ref⁻¹ * q_cur
        rot_delta = rot_ref.inv() * rot_cur

        # 4. 旋转加速度和角速度
        acc_raw = np.array([msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z])
        acc_corr = rot_delta.apply(acc_raw)
        imu_msg.linear_acceleration.x = acc_corr[0]
        imu_msg.linear_acceleration.y = acc_corr[1]
        imu_msg.linear_acceleration.z = acc_corr[2]

        gyro_raw = np.array([msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z])
        gyro_corr = rot_delta.apply(gyro_raw)
        imu_msg.angular_velocity.x = gyro_corr[0]
        imu_msg.angular_velocity.y = gyro_corr[1]
        imu_msg.angular_velocity.z = gyro_corr[2]

        imu_msg.orientation.w = q_ref[0]
        imu_msg.orientation.x = q_ref[1]
        imu_msg.orientation.y = q_ref[2]
        imu_msg.orientation.z = q_ref[3]
        return imu_msg

    def align_callback(self, request, response):
        """每收到一次请求，执行此回调一次"""
        if request.request_align:
            if all(self.imu_muti_bool):
                response.success = True
                response.message = "already aligned"
                self.if_align = True
                self.latest_delta = request.delta

            else:
                response.success = False
                response.message = "aligning..."
                self.get_logger().info("正在对齐 muti-IMU，请稍候...")
                self.if_align = False
        else:
            response.success = True
            response.message = "nothing done"
            self.if_align = False
        return response
    
    def imu0_data_received_callback(self, msg: Imu):
        """
        收到 /imu0/data_raw 消息发送tf的回调函数。
        """
        time_stamp = Clock().now()
        imu_msg_0 = Imu()
        imu_msg_0.header.stamp = time_stamp.to_msg()
        imu_msg_0.header.frame_id = f"imu0_link"
        imu_msg_0.linear_acceleration.x = msg.linear_acceleration.x * 1.0
        imu_msg_0.linear_acceleration.y = msg.linear_acceleration.y * 1.0
        imu_msg_0.linear_acceleration.z = msg.linear_acceleration.z * 1.0
        imu_msg_0.angular_velocity.x = msg.angular_velocity.x * 1.0
        imu_msg_0.angular_velocity.y = msg.angular_velocity.y * 1.0
        imu_msg_0.angular_velocity.z = msg.angular_velocity.z * 1.0
        imu_msg_0.orientation.w = msg.orientation.w
        imu_msg_0.orientation.x = msg.orientation.x
        imu_msg_0.orientation.y = msg.orientation.y
        imu_msg_0.orientation.z = msg.orientation.z
        if self.if_align:
            imu_msg_0_align = self.apply_alignment(0, imu_msg_0)
            self.get_logger().info("已经对齐imu0")
        else:
            imu_msg_0_align = imu_msg_0

        current_accel_0 = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        if self.last_accel_0 is not None:
            delta_accel_0 = current_accel_0 - self.last_accel_0  # 一个 3x1 的向量
            if delta_accel_0 @ delta_accel_0 > 0.01:  # 如果加速度变化超过某个阈值，认为有动作
                self.get_logger().info(f"检测到动作，imu0_delta_accel: {delta_accel_0}")
                self.imu_muti_bool[0] = False  # 标记 imu0 未对齐
            else:
                self.imu_muti_bool[0] = True  # 标记 imu0 已对齐
                self.imu_publishers_fix[0] = np.array([
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ])
                
        self.last_accel_0 = current_accel_0
        self.aligned_pub_0.publish(imu_msg_0_align)


    def imu1_data_received_callback(self, msg: Imu):
        """
        收到 /imu1/data_raw 消息发送tf的回调函数。
        """
        time_stamp = Clock().now()
        imu_msg_1 = Imu()
        imu_msg_1.header.stamp = time_stamp.to_msg()
        imu_msg_1.header.frame_id = f"imu1_link"
        imu_msg_1.linear_acceleration.x = msg.linear_acceleration.x * 1.0
        imu_msg_1.linear_acceleration.y = msg.linear_acceleration.y * 1.0
        imu_msg_1.linear_acceleration.z = msg.linear_acceleration.z * 1.0
        imu_msg_1.angular_velocity.x = msg.angular_velocity.x * 1.0
        imu_msg_1.angular_velocity.y = msg.angular_velocity.y * 1.0
        imu_msg_1.angular_velocity.z = msg.angular_velocity.z * 1.0
        imu_msg_1.orientation.w = msg.orientation.w
        imu_msg_1.orientation.x = msg.orientation.x
        imu_msg_1.orientation.y = msg.orientation.y
        imu_msg_1.orientation.z = msg.orientation.z
        if self.if_align:
            imu_msg_1_align = self.apply_alignment(1, imu_msg_1)
            self.get_logger().info("已经对齐imu1")
        else:
            imu_msg_1_align = imu_msg_1
        
        current_accel_1 = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        if self.last_accel_1 is not None:
            delta_accel_1 = current_accel_1 - self.last_accel_1  # 一个 3x1 的向量
            if delta_accel_1 @ delta_accel_1 > 0.01:  # 如果加速度变化超过某个阈值，认为有动作
                self.get_logger().info(f"检测到动作，imu1_delta_accel: {delta_accel_1}")
                self.imu_muti_bool[1] = False  # 标记 imu1 未对齐
            else:
                self.imu_muti_bool[1] = True  # 标记 imu1 已对齐
                self.imu_publishers_fix[1] = np.array([
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ])
        self.last_accel_1 = current_accel_1
        self.aligned_pub_1.publish(imu_msg_1_align)



    def imu2_data_received_callback(self, msg: Imu):
        """
        收到 /imu2/data/filtered 消息发送tf的回调函数。
        """
        time_stamp = Clock().now()
        imu_msg_2 = Imu()
        imu_msg_2.header.stamp = time_stamp.to_msg()
        imu_msg_2.header.frame_id = f"imu2_link"
        imu_msg_2.linear_acceleration.x = msg.linear_acceleration.x * 1.0
        imu_msg_2.linear_acceleration.y = msg.linear_acceleration.y * 1.0
        imu_msg_2.linear_acceleration.z = msg.linear_acceleration.z * 1.0
        imu_msg_2.angular_velocity.x = msg.angular_velocity.x * 1.0
        imu_msg_2.angular_velocity.y = msg.angular_velocity.y * 1.0
        imu_msg_2.angular_velocity.z = msg.angular_velocity.z * 1.0
        imu_msg_2.orientation.w = msg.orientation.w
        imu_msg_2.orientation.x = msg.orientation.x
        imu_msg_2.orientation.y = msg.orientation.y
        imu_msg_2.orientation.z = msg.orientation.z
        if self.if_align:
            imu_msg_2_align = self.apply_alignment(2, imu_msg_2)
            self.get_logger().info("已经对齐imu2")
        else:
            imu_msg_2_align = imu_msg_2
        
        current_accel_2 = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        if self.last_accel_2 is not None:
            delta_accel_2 = current_accel_2 - self.last_accel_2  # 一个 3x1 的向量
            if delta_accel_2 @ delta_accel_2 > 0.01:  # 如果加速度变化超过某个阈值，认为有动作
                self.get_logger().info(f"检测到动作，imu2_delta_accel: {delta_accel_2}")
                self.imu_muti_bool[2] = False  # 标记 imu2 未对齐
            else:
                self.imu_muti_bool[2] = True  # 标记 imu2 已对齐
                self.imu_publishers_fix[2] = np.array([
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ])
        self.last_accel_2 = current_accel_2
        self.aligned_pub_2.publish(imu_msg_2_align)

def main(args=None):
    rclpy.init(args=args)
    node = AlignService()
    rclpy.spin(node)          # 保持运行，处理请求
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
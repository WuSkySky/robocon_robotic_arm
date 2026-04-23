import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interface.msg import UnloadedSerialData
from sensor_msgs.msg import Imu

class PubTargetPoseNode(Node):
    def __init__(self):
        super().__init__('pub_target_pose_node')

        self.delta_time_process_data = 0.01
        self.last_pose=Pose()  # 用于存储上一次的位姿数据
        self.latest_imu_data = None #用与存储IMU数据
        self.unloaded_serial_data = None #存储串口数据
        # sub
        self.sub_serial = self.create_subscription(
            UnloadedSerialData,
            '/unloaded_serial_data',
            self.unloaded_serial_data_received_callback,
            10
        )

        # sub_imu
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu/data',          
            self.imu_data_callback,
            10
        )   

        # pub
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

        # timer
        self.timer = self.create_timer(self.delta_time_process_data, self.process_data_callback)

    # 回调函数
    def unloaded_serial_data_received_callback(self, msg):
        self.unloaded_serial_data = msg

    def imu_data_callback(self, msg):
        self.latest_imu = msg    

    def process_data_callback(self):
        if not hasattr(self, 'unloaded_serial_data'):
            return
        
        if not hasattr(self, 'latest_imu'):
            return

        self.pub_pose(self.unloaded_serial_data,self.latest_imu)

    def pub_pose(self,data,data_imu):
        speed_factor=self.delta_time_process_data*0.00015
        pose_msg = Pose()

        remote_input = np.array([
            data.rc_left_y * speed_factor,    # 遥控器左摇杆y控制x轴移动
            -data.rc_left_x * speed_factor,    # 遥控器左摇杆x的相反数控制y轴移动
            data.rc_right_y * speed_factor    # 遥控器右摇杆y控制z轴移动
        ])

        data_imu = data_imu.orientation
        # 计算旋转矩阵
        R = np.array([
            [1 - 2*data_imu.y*data_imu.y - 2*data_imu.z*data_imu.z, 2*data_imu.x*data_imu.y - 2*data_imu.z*data_imu.w, 2*data_imu.x*data_imu.z + 2*data_imu.y*data_imu.w],
            [2*data_imu.x*data_imu.y + 2*data_imu.z*data_imu.w, 1 - 2*data_imu.x*data_imu.x - 2*data_imu.z*data_imu.z, 2*data_imu.y*data_imu.z - 2*data_imu.x*data_imu.w],
            [2*data_imu.x*data_imu.z - 2*data_imu.y*data_imu.w, 2*data_imu.y*data_imu.z + 2*data_imu.x*data_imu.w, 1 - 2*data_imu.x*data_imu.x - 2*data_imu.y*data_imu.y]
        ])
        
        # 转换坐标系
        world_movement = R @ remote_input

        # 更新位置
        pose_msg.position.x = self.last_pose.position.x + world_movement[0]
        pose_msg.position.y = self.last_pose.position.y + world_movement[1]
        pose_msg.position.z = self.last_pose.position.z + world_movement[2]
        pose_msg.orientation.w = data_imu.w
        pose_msg.orientation.x = data_imu.x
        pose_msg.orientation.y = data_imu.y
        pose_msg.orientation.z = data_imu.z
        self.pose_publisher.publish(pose_msg)
        self.last_pose = pose_msg



def main(args=None):
    rclpy.init(args=args)
    node = PubTargetPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


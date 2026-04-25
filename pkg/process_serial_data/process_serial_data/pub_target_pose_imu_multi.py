#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion         
from interface.msg import UnloadedSerialData
from sensor_msgs.msg import Imu


class PubTargetPoseNode_multi(Node):
    def __init__(self):
        super().__init__('pub_target_pose_node_multi')

        self.declare_parameter('imu_topics', ['/imu0/data', '/imu1/data', '/imu2/data'])
        self.imu_topics = self.get_parameter('imu_topics').get_parameter_value().string_array_value

        self.delta_time_process_data = 0.01
        self.last_pose = Pose()                         # 用于存储上一次的位姿数据
        self.latest_imu_data = [None] * len(self.imu_topics)
        self.unloaded_serial_data = None                # 存储串口数据

        # sub
        self.sub_serial = self.create_subscription(
            UnloadedSerialData,
            '/unloaded_serial_data',
            self.unloaded_serial_data_received_callback,
            10
        )

        # sub_imu_multi
        for idx, topic in enumerate(self.imu_topics):
            self.create_subscription(
                Imu, topic,
                lambda msg, i=idx: self.imu_data_callback(msg, i),   
                10
            )
            self.get_logger().info(f"Subscribed to IMU topic: {topic}")

        # pub
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

        # timer
        self.timer = self.create_timer(self.delta_time_process_data, self.process_data_callback)

    # ---------- 回调函数 ----------
    def unloaded_serial_data_received_callback(self, msg):
        self.unloaded_serial_data = msg
        self.start_process_data_flag = True

    def imu_data_callback(self, msg, idx):
        """存储各 IMU 的最新数据"""
        self.latest_imu_data[idx] = msg

    # ---------- 定时处理（覆盖原单一回调，直接实现多 IMU 融合）----------
    def process_data_callback(self):
        """定时器回调：融合多 IMU 并计算位姿"""
        # 检查串口数据就绪
        if self.unloaded_serial_data is None:
            return

        # 收集所有有效的 IMU 数据
        valid_imus = [imu for imu in self.latest_imu_data if imu is not None]
        if not valid_imus:
            return

        # 融合多个 IMU 的姿态（四元数平均）
        fused_orientation = self.average_quaternions([imu.orientation for imu in valid_imus])

        # 调用位姿计算并发布
        self.pub_pose(self.unloaded_serial_data, fused_orientation)

    def pub_pose(self, data, fused_orientation):
        left_x = float(data.rc_left_x) / 1000.0
        left_y = float(data.rc_left_y) / 1000.0
        right_y = float(data.rc_right_y) / 1000.0

        speed_factor = self.delta_time_process_data * 0.0001
        pose_msg = Pose()

        remote_input = np.array([
            left_y * speed_factor,    # 左摇杆 Y 轴 -> 前进/后退 (机体 X)
            -left_x * speed_factor,   # 左摇杆 X 轴取反 -> 左/右平移 (机体 Y)
            right_y * speed_factor    # 右摇杆 Y 轴 -> 上升/下降 (机体 Z)
        ])
        
        q = fused_orientation

        # 计算旋转矩阵
        R = np.array([
            [1 - 2*q.y*q.y - 2*q.z*q.z,   2*q.x*q.y - 2*q.z*q.w,   2*q.x*q.z + 2*q.y*q.w],
            [2*q.x*q.y + 2*q.z*q.w,       1 - 2*q.x*q.x - 2*q.z*q.z, 2*q.y*q.z - 2*q.x*q.w],
            [2*q.x*q.z - 2*q.y*q.w,       2*q.y*q.z + 2*q.x*q.w,   1 - 2*q.x*q.x - 2*q.y*q.y]
        ])

        # 转换坐标系：机体速度 -> 世界速度
        world_movement = R @ remote_input

        # 更新位置
        pose_msg.position.x = self.last_pose.position.x + world_movement[0]
        pose_msg.position.y = self.last_pose.position.y + world_movement[1]
        pose_msg.position.z = self.last_pose.position.z + world_movement[2]
        pose_msg.orientation = q

        self.pose_publisher.publish(pose_msg)
        self.last_pose = pose_msg

    @staticmethod
    def average_quaternions(quaternions):
        """对多个四元数进行平均（最小特征向量法）"""
        if not quaternions:
            return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        Q = np.zeros((4, 4))
        for q in quaternions:
            v = np.array([q.w, q.x, q.y, q.z])
            Q += np.outer(v, v)

        eigvals, eigvecs = np.linalg.eig(Q)
        max_idx = np.argmax(eigvals)
        avg_vec = eigvecs[:, max_idx]

        return Quaternion(
            w=float(avg_vec[0]),
            x=float(avg_vec[1]),
            y=float(avg_vec[2]),
            z=float(avg_vec[3])
        )


def main(args=None):
    rclpy.init(args=args)
    node = PubTargetPoseNode_multi()   
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
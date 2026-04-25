#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray
from YbImuLib import YbImuSerial
import glob


class MultiImuDriver(Node):
    """
    多 IMU 驱动节点：同时管理并发布多个 YbImu 传感器的数据。
    每个 IMU 拥有独立的话题命名（imu0/data, imu1/data, ...）。
    """

    def __init__(self):
        super().__init__('multi_imu_driver')
        self.robots = []              # 存储 YbImuSerial 对象列表
        self.imu_publishers = []      # 存储每个 IMU 的发布者字典（避免与基类属性冲突）
        self.init_devices()           # 初始化所有设备

        if not self.robots:
            self.get_logger().error("No IMU devices could be opened. Node will exit.")
            return

        # 创建定时器，周期 0.1 秒
        self.timer = self.create_timer(0.1, self.publish_all_data)

    def init_devices(self):
        """尝试打开所有指定的串口，并为每个成功设备创建发布者"""
        #port_list = ["/dev/myimu", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]

        port_list = sorted(glob.glob("/dev/imu*"))
        if not port_list:
            port_list = sorted(glob.glob("/dev/ttyUSB*"))


        for idx, port in enumerate(port_list):
            try:
                robot = YbImuSerial(port)
                robot.create_receive_threading()
                self.robots.append(robot)
                self.get_logger().info(f"Opened IMU {idx} on {port}")

                # 为该 IMU 创建专属发布者（话题名包含索引）
                pub_dict = {
                    'imu': self.create_publisher(Imu, f"imu{idx}/data_raw", 100),
                    'mag': self.create_publisher(MagneticField, f"imu{idx}/mag", 100),
                    'baro': self.create_publisher(Float32MultiArray, f"imu{idx}/baro", 100),
                    'euler': self.create_publisher(Float32MultiArray, f"imu{idx}/euler", 100),
                }
                self.imu_publishers.append(pub_dict)

            except Exception as e:
                self.get_logger().warn(f"Failed to open {port}: {e}")

        if not self.robots:
            self.get_logger().error("No IMU devices were opened successfully.")

    def publish_all_data(self):
        """定时回调：遍历所有 IMU，读取数据并发布"""
        time_stamp = Clock().now()

        for idx, robot in enumerate(self.robots):
            try:
                # 读取传感器数据
                ax, ay, az = robot.get_accelerometer_data()
                gx, gy, gz = robot.get_gyroscope_data()
                mx, my, mz = robot.get_magnetometer_data()
                q0, q1, q2, q3 = robot.get_imu_quaternion_data()
                height, temperature, pressure, pressure_contrast = robot.get_baro_data()
                roll, pitch, yaw = robot.get_imu_attitude_data(True)

                # --- 构造 IMU 消息 ---
                imu_msg = Imu()
                imu_msg.header.stamp = time_stamp.to_msg()
                imu_msg.header.frame_id = f"imu{idx}_link"
                imu_msg.linear_acceleration.x = ax * 1.0
                imu_msg.linear_acceleration.y = ay * 1.0
                imu_msg.linear_acceleration.z = az * 1.0
                imu_msg.angular_velocity.x = gx * 1.0
                imu_msg.angular_velocity.y = gy * 1.0
                imu_msg.angular_velocity.z = gz * 1.0
                imu_msg.orientation.w = q0
                imu_msg.orientation.x = q1
                imu_msg.orientation.y = q2
                imu_msg.orientation.z = q3

                mag_msg = MagneticField()
                mag_msg.header.stamp = time_stamp.to_msg()
                mag_msg.header.frame_id = f"imu{idx}_link"
                mag_msg.magnetic_field.x = mx * 1.0
                mag_msg.magnetic_field.y = -my * 1.0
                mag_msg.magnetic_field.z = mz * 1.0

                baro_msg = Float32MultiArray()
                baro_msg.data = [height, temperature, pressure, pressure_contrast]

                euler_msg = Float32MultiArray()
                euler_msg.data = [roll, pitch, yaw]

                 
                pubs = self.imu_publishers[idx]
                pubs['imu'].publish(imu_msg)
                pubs['mag'].publish(mag_msg)
                pubs['baro'].publish(baro_msg)
                pubs['euler'].publish(euler_msg)

            except Exception as e:
                self.get_logger().error(f"Error reading/publishing IMU {idx}: {e}")

    def ready(self):
        """返回是否有至少一个 IMU 成功连接"""
        return len(self.robots) > 0


def main(args=None):
    rclpy.init(args=args)
    node = MultiImuDriver()
    if not node.ready():
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
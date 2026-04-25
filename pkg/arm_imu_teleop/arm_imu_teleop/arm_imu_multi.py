#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

class MultiImuTfPublisher(Node):

    def __init__(self):
        super().__init__('multi_imu_tf_publisher')
        self.br = TransformBroadcaster(self)
        # 配置三个IMU的参数
        self.configs = [
            {'topic': '/imu0/data', 'length': 0.30, 'axis': 'x', 
             'parent': 'shoulder_link', 'child': 'elbow_tip'},
            {'topic': '/imu1/data', 'length': 0.25, 'axis': 'z', 
             'parent': 'elbow_tip', 'child': 'wrist_tip'},
            {'topic': '/imu2/data', 'length': 0.10, 'axis': 'x', 
             'parent': 'wrist_tip', 'child': 'tool_tip'}
        ]
        
        # 为每个IMU创建订阅者
        self.subs = []
        for i, cfg in enumerate(self.configs):
            # 使用闭包捕获当前配置
            sub = self.create_subscription(
                Imu, cfg['topic'], 
                lambda msg, idx=i: self.imu_cb(msg, idx), 10)
            self.subs.append(sub)
            self.get_logger().info(f'Subscribed to {cfg["topic"]}')

    def imu_cb(self, msg: Imu, idx: int):
        cfg = self.configs[idx]
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        
        rot = R.from_quat(quat)
        rot_matrix = rot.as_matrix()
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        col = rot_matrix[:, axis_map[cfg['axis']]]
        trans = col * cfg['length']
        
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = cfg['parent']
        t.child_frame_id = cfg['child']
        
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation = q
        
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MultiImuTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
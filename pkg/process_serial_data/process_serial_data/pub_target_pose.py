import rclpy
import struct
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interface.msg import UnloadedSerialData

class PubTargetPoseNode(Node):
    def __init__(self):
        super().__init__('pub_target_pose_node')

        self.delta_time_process_data = 0.01

        self.last_pose=Pose()  # 用于存储上一次的位姿数据

        # sub
        self.sub_serial = self.create_subscription(
            UnloadedSerialData,
            '/unloaded_serial_data',
            self.unloaded_serial_data_received_callback,
            10
        )

        # pub
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

        # timer
        self.timer = self.create_timer(self.delta_time_process_data, self.process_data_callback)

    # 回调函数
    def unloaded_serial_data_received_callback(self, msg):
        self.unloaded_serial_data = msg
        self.start_process_data_flag = True
        
    def process_data_callback(self):
        if not hasattr(self, 'unloaded_serial_data'):
            return
            
        self.pub_pose(self.unloaded_serial_data)

    def pub_pose(self,data):
        speed_factor=self.delta_time_process_data*0.0001
        pose_msg = Pose()
        pose_msg.position.x = self.last_pose.position.x + data.rc_left_y*speed_factor # 遥控器左摇杆y控制x轴移动
        pose_msg.position.y = self.last_pose.position.y - data.rc_left_x*speed_factor # 遥控器左摇杆x的相反数控制y轴移动
        pose_msg.position.z = self.last_pose.position.z + data.rc_right_y*speed_factor # 遥控器右摇杆y控制z轴移动
        pose_msg.orientation.w = data.w
        pose_msg.orientation.x = data.x
        pose_msg.orientation.y = data.y
        pose_msg.orientation.z = data.z
        self.pose_publisher.publish(pose_msg)
        self.last_pose = pose_msg
            
def main(args=None):
    rclpy.init(args=args)
    node = PubTargetPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


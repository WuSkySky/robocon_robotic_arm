import rclpy
import struct
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8MultiArray

class MinimalNode(Node):
    def __init__(self):
        super().__init__('unload_serial_data_node')

        self.start_process_data_flag = False

        self.delta_time_process_data = 0.01

        self.last_pose=Pose()  # 用于存储上一次的位姿数据

        # sub
        self.sub_serial = self.create_subscription(
            UInt8MultiArray,
            '/serial_read',
            self.serial_received_callback,
            10
        )

        # pub
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

        # timer
        self.timer = self.create_timer(self.delta_time_process_data, self.process_data_callback)

    # 回调函数
    def serial_received_callback(self, msg):
        self.msg_raw = msg.data
        self.start_process_data_flag = True
        
    def process_data_callback(self):
        if not self.start_process_data_flag:
            return
        #解析数据帧头尾部，不对直接丢
        if (self.msg_raw[0] != 85 or self.msg_raw[1] != 170 or self.msg_raw[62] != 13 or self.msg_raw[63] != 10):
            return 
        
        unpacked_data=self.data_unpack(self.msg_raw)
        self.pub_pose(unpacked_data)


    def data_unpack(self, data):
        #四元数 w,x,y,z
        w_ = self.unpack_tofloat32(data[2:6])
        x_ = self.unpack_tofloat32(data[6:10])
        y_ = self.unpack_tofloat32(data[10:14])
        z_ = self.unpack_tofloat32(data[14:18])

        #遥控器右摇杆x,y 左摇杆x,y 遥控器拨轮
        rc_right_x = self.unpack_toint16(data[18:20])
        rc_right_y = self.unpack_toint16(data[20:22])
        rc_left_x = self.unpack_toint16(data[22:24])
        rc_left_y = self.unpack_toint16(data[24:26])

        # print(f"rc_right_x: {rc_right_x}, rc_right_y: {rc_right_y}, rc_left_x: {rc_left_x}, rc_left_y: {rc_left_y}")
        
        #遥控器拨轮
        rc_dial = self.unpack_toint16(data[26:28])

        #左，右拨动开关
        rc_switch_left = data[28]
        rc_switch_right = data[29]

        return {
            'quaternion': (w_, x_, y_, z_),
            'rc_right': (rc_right_x, rc_right_y),
            'rc_left': (rc_left_x, rc_left_y),
            'rc_dial': rc_dial,
            'rc_switch': (rc_switch_left, rc_switch_right)
        }

    def pub_pose(self,unpacked_data):
        speed_factor=self.delta_time_process_data*0.0001
        pose_msg = Pose()
        pose_msg.position.x = self.last_pose.position.x + unpacked_data['rc_left'][0]*speed_factor
        pose_msg.position.y = self.last_pose.position.y + unpacked_data['rc_left'][1]*speed_factor
        pose_msg.position.z = self.last_pose.position.z + unpacked_data['rc_right'][1]*speed_factor
        pose_msg.orientation.w = unpacked_data['quaternion'][0]
        pose_msg.orientation.x = unpacked_data['quaternion'][1]
        pose_msg.orientation.y = unpacked_data['quaternion'][2]
        pose_msg.orientation.z = unpacked_data['quaternion'][3]
        self.pose_publisher.publish(pose_msg)
        self.last_pose = pose_msg

    #解析int_arr转换为float32
    def unpack_tofloat32(self, int_arr):
        b0, b1, b2, b3 = int_arr
        int_val = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
        float_val = struct.unpack('<f', bytes(int_arr))[0]
        return float_val

    #解析int_arr转换为int16
    def unpack_toint16(self, int_arr):
        return struct.unpack('<h', bytes(int_arr))[0]
            
def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


import rclpy
from rclpy.node import Node
from interface.msg import UnloadedSerialData
from std_msgs.msg import Float64

class PubGripperControlNode(Node):
    def __init__(self):
        super().__init__('pub_gripper_control_node')

        self.delta_time_process_data = 0.01
        self.recoded_gripper_control_msg=Float64()

        # sub
        self.sub_serial = self.create_subscription(
            UnloadedSerialData,
            '/unloaded_serial_data',
            self.unloaded_serial_data_received_callback,
            10
        )

        # pub
        self.gripper_control_publisher = self.create_publisher(Float64, '/gripper_control', 10)

        # timer
        self.timer = self.create_timer(self.delta_time_process_data, self.process_data_callback)

    # 回调函数
    def unloaded_serial_data_received_callback(self, msg):
        self.unloaded_serial_data = msg

    def process_data_callback(self):
        if not hasattr(self, 'unloaded_serial_data'):
            return

        control_factor = -self.delta_time_process_data*0.0005
        self.recoded_gripper_control_msg.data += self.unloaded_serial_data.rc_dial * control_factor

        # 范围限制
        if(self.recoded_gripper_control_msg.data > 0.07):
            self.recoded_gripper_control_msg.data = 0.07
        elif(self.recoded_gripper_control_msg.data < 0.001):
            self.recoded_gripper_control_msg.data = 0.001

        self.gripper_control_publisher.publish(self.recoded_gripper_control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubGripperControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


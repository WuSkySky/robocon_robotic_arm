import rclpy
from rclpy.node import Node
from filter.filter import SecondOrderButterworthLowPass
from sensor_msgs.msg import Imu

class imu_filter_node(Node):
    def __init__(self, name, filter_topic, output_topic):
        super().__init__(name)

        # sub
        self.sub_filter_topic = self.create_subscription(
            Imu,
            filter_topic,
            self.receive_imu_data_callback,
            10
        )

        # pub
        self.pub_output_topic = self.create_publisher(Imu, output_topic, 10)

        self.filter = SecondOrderButterworthLowPass(Wn=0.25)
    
    def receive_imu_data_callback(self, msg):
        
        orientation = self.filter.filter_multiple([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        
        filtered_imu = Imu()
        filtered_imu.header.frame_id = 'imu_link'
        filtered_imu.orientation.w = orientation[0]
        filtered_imu.orientation.x = orientation[1]
        filtered_imu.orientation.y = orientation[2]
        filtered_imu.orientation.z = orientation[3]

        self.pub_output_topic.publish(filtered_imu)


def main(args=None):
    rclpy.init(args=args)
    node = imu_filter_node("imu_filter_node", "/imu/data", "/imu/data/filtered")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()

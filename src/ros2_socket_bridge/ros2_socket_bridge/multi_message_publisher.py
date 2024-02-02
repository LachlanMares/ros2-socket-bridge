import rclpy

from rclpy.node import Node
from std_msgs.msg import String, Float32, UInt8
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessagePublisherNode(Node):
    def __init__(self):
        super().__init__('multi_message_publisher_node')
        # Topics
        self.uint8_topic = '/uint8'
        self.float32_topic = '/float32'
        self.string_topic = "/string"
        self.odom_topic = "/odometry"
        self.pose_topic = "/pose"
        self.imu_topic = "/imu"

        # Publishers
        self.string_publisher = self.create_publisher(String, self.string_topic, 10)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.imu_publisher = self.create_publisher(Imu, self.imu_topic, 10)
        self.uint8_publisher = self.create_publisher(UInt8, self.uint8_topic, 10)
        self.float32_publisher = self.create_publisher(Float32, self.float32_topic, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        self.string_publisher.publish(String()) 
        self.odom_publisher.publish(Odometry()) 
        self.pose_publisher.publish(PoseWithCovarianceStamped()) 
        self.imu_publisher.publish(Imu()) 
        self.uint8_publisher.publish(UInt8()) 
        self.float32_publisher.publish(Float32()) 

def main(args=None):
    rclpy.init(args=args)

    node = MultiMessagePublisherNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
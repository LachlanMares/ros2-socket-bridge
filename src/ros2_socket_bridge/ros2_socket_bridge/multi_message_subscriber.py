import rclpy

from rclpy.node import Node
from std_msgs.msg import String, Float32, UInt8
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessageSubscriberNode(Node):
    def __init__(self):
        super().__init__('multi_message_subscriber_node')
        # Topics
        self.uint8_topic = '/uint8'
        self.float32_topic = '/float32'
        self.string_topic = "/string"
        self.odom_topic = "/odometry"
        self.pose_topic = "/pose"
        self.imu_topic = "/imu"

        # Subscribers
        self.string_subscriber = self.create_subscription(String, '/client' + self.string_topic, self.string_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/client' + self.odom_topic, self.odometry_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/client' + self.pose_topic, self.pose_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/client' + self.imu_topic, self.imu_callback, 10)
        self.uint8_subscriber = self.create_subscription(UInt8, '/server' + self.uint8_topic, self.uint8_callback, 10)
        self.float32_subscriber = self.create_subscription(Float32, '/server' + self.float32_topic, self.float32_callback, 10)

    def string_callback(self, msg):
        self.get_logger().info('String')

    def odometry_callback(self, msg):
        self.get_logger().info('Odometry')

    def pose_callback(self, msg):
        self.get_logger().info('Pose')

    def imu_callback(self, msg):
        self.get_logger().info('Imu')

    def uint8_callback(self, msg):
        self.get_logger().info('Uint8')
    
    def float32_callback(self, msg):
        self.get_logger().info('Float32')

def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MultiMessageSubscriberNode()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
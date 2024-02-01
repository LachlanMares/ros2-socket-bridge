import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessageSubscriberNode(Node):
    def __init__(self):
        super().__init__('multi_message_subscriber_node')
        self.string_subscriber = self.create_subscription(String, '/client/string', self.string_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/client/odometry', self.odometry_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/client/pose', self.pose_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/client/imu', self.imu_callback, 10)

    def string_callback(self, msg):
        self.get_logger().info('String')

    def odometry_callback(self, msg):
        self.get_logger().info('Odometry')

    def pose_callback(self, msg):
        self.get_logger().info('Pose')

    def imu_callback(self, msg):
        self.get_logger().info('Imu')
    

def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MultiMessageSubscriberNode()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
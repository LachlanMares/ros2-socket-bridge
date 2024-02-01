import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessagePublisherNode(Node):
    def __init__(self):
        super().__init__('multi_message_publisher_node')
        self.string_publisher = self.create_publisher(String, '/string', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odometry', 10)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/pose', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        self.string_publisher.publish(String()) 
        self.odom_publisher.publish(Odometry()) 
        self.pose_publisher.publish(PoseWithCovarianceStamped()) 
        self.imu_publisher.publish(Imu()) 

def main(args=None):
    rclpy.init(args=args)

    node = MultiMessagePublisherNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
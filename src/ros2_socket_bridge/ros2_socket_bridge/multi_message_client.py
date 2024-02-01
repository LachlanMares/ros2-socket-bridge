import rclpy
import pickle
import zmq

from threading import Thread

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessageClientNode(Node):

    def __init__(self):
        super().__init__('multi_message_client_node')
        context = zmq.Context()

        self.send_socket = context.socket(zmq.PAIR)
        self.send_socket.connect(f"tcp://localhost:7011")
        
        self.recieve_socket = context.socket(zmq.PAIR)
        self.recieve_socket.connect(f"tcp://localhost:7010")
        self.recieve_socket.RCVTIMEO = 5000  # Milliseconds

        self.recieve_topic_string = "/string"
        self.string_publisher = self.create_publisher(String, self.recieve_topic_string, 10)

        self.recieve_topic_odom = "/odometry"
        self.odom_publisher = self.create_publisher(Odometry, self.recieve_topic_odom, 10)

        self.recieve_topic_pose = "/pose"
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.recieve_topic_pose, 10)

        self.recieve_topic_imu = "/imu"
        self.imu_publisher = self.create_publisher(Imu, self.recieve_topic_imu, 10)

        self.send_loop_running = False
        self.send_loop_thread = Thread(target=self.send_loop)

        self.recieve_loop_running = False
        self.recieve_loop_thread = Thread(target=self.recieve_loop)

    def start(self, ):
        self.send_loop_running = True
        self.recieve_loop_running = True

        self.send_loop_thread.start()
        self.recieve_loop_thread.start()

    def stop(self, ):
        self.send_loop_running = False
        self.recieve_loop_running = False
    
        self.send_loop_thread.join()        
        self.recieve_loop_thread.join()

    def recieve_loop(self, ):
        while self.recieve_loop_running:  # Where is the equivalent to rospy.is_shutdown()?
            try:
                message_dict = pickle.loads(self.recieve_socket.recv())
                
                if message_dict["topic"] == self.recieve_topic_string:
                    string_thread = Thread(target=self.publish_string_msg, args=(message_dict["payload"], ))
                    string_thread.start()

                elif message_dict["topic"] == self.recieve_topic_odom:
                    odometry_thread = Thread(target=self.publish_odometry_msg, args=(message_dict["payload"], ))
                    odometry_thread.start()
                
                elif message_dict["topic"] == self.recieve_topic_pose:
                    pose_thread = Thread(target=self.publish_pose_msg, args=(message_dict["payload"], ))
                    pose_thread.start()
                
                elif message_dict["topic"] == self.recieve_topic_imu:
                    imu_thread = Thread(target=self.publish_imu_msg, args=(message_dict["payload"], ))
                    imu_thread.start()

            except Exception as e:
                if "Resource temporarily unavailable" in str(e):  # Need to set a timeout or the script hangs on the .recv() method
                    pass

                else:
                    self.get_logger().info(str(e))
    
    def send_loop(self, ):
        # TODO: something useful
        pass

    def publish_string_msg(self, msg):
        self.string_publisher.publish(msg) 

    def publish_odometry_msg(self, msg):
        self.odom_publisher.publish(msg)

    def publish_pose_msg(self, msg):
        self.pose_publisher.publish(msg)

    def publish_imu_msg(self, msg):
        self.imu_publisher.publish(msg) 
    

def main(args=None):
    rclpy.init(args=args)

    multi_message_client_node = MultiMessageClientNode()
    multi_message_client_node.start()

    rclpy.spin(multi_message_client_node)

    multi_message_client_node.stop()
    multi_message_client_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
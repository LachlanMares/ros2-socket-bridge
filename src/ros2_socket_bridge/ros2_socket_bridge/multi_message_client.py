import rclpy
import pickle
import zmq
import queue

from threading import Thread

from rclpy.node import Node
from std_msgs.msg import String, Float32, UInt8
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessageClientNode(Node):

    def __init__(self):
        super().__init__('multi_message_client_node')

        # Queue
        self.message_queue = queue.Queue(maxsize=16)

        # ZMQ
        context = zmq.Context()
        self.send_socket = context.socket(zmq.PAIR)
        self.send_socket.connect(f"tcp://localhost:7011")
        
        self.recieve_socket = context.socket(zmq.PAIR)
        self.recieve_socket.connect(f"tcp://localhost:7010")
        self.recieve_socket.RCVTIMEO = 10  # Milliseconds

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

        # Subscribers
        self.uint8_subscriber = self.create_subscription(UInt8, self.uint8_topic, self.uint8_callback, 10)
        self.float32_subscriber = self.create_subscription(Float32, self.float32_topic, self.float32_callback, 10)

        # Timers
        self.send_loop_lock = False
        self.recieve_loop_lock = False
        self.send_loop_timer = self.create_timer(0.002, self.send_loop)
        self.recieve_loop_timer = self.create_timer(0.002, self.recieve_loop)

    def recieve_loop(self, ):
        if not self.recieve_loop_lock:  
            self.recieve_loop_lock = True

            try:
                message_dict = pickle.loads(self.recieve_socket.recv())
                
                if message_dict["topic"] == self.string_topic:
                    string_thread = Thread(target=self.publish_string_msg, args=(message_dict["payload"], ))
                    string_thread.start()

                elif message_dict["topic"] == self.odom_topic:
                    odometry_thread = Thread(target=self.publish_odometry_msg, args=(message_dict["payload"], ))
                    odometry_thread.start()
                
                elif message_dict["topic"] == self.pose_topic:
                    pose_thread = Thread(target=self.publish_pose_msg, args=(message_dict["payload"], ))
                    pose_thread.start()
                
                elif message_dict["topic"] == self.imu_topic:
                    imu_thread = Thread(target=self.publish_imu_msg, args=(message_dict["payload"], ))
                    imu_thread.start()

            except Exception as e:
                if "Resource temporarily unavailable" in str(e):  # Need to set a timeout or the script hangs on the .recv() method
                    pass

                else:
                    self.get_logger().info(str(e))
            
            self.recieve_loop_lock = False
    
    def send_loop(self, ):
        if not self.send_loop_lock:  
            
            self.send_loop_lock = True

            try:
                queue_data = self.message_queue.get(timeout=0.001)
                self.send_socket.send(queue_data)

            except queue.Empty:
                pass

            except Exception as e:
                self.get_logger().info(e)
            
            self.send_loop_lock = False

    def publish_string_msg(self, msg):
        self.string_publisher.publish(msg) 

    def publish_odometry_msg(self, msg):
        self.odom_publisher.publish(msg)

    def publish_pose_msg(self, msg):
        self.pose_publisher.publish(msg)

    def publish_imu_msg(self, msg):
        self.imu_publisher.publish(msg) 
    
    def uint8_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": self.uint8_topic, "payload": msg}), block=False)
        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def float32_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": self.float32_topic, "payload": msg}), block=False)
        
        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)

    multi_message_client_node = MultiMessageClientNode()

    rclpy.spin(multi_message_client_node)

    multi_message_client_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
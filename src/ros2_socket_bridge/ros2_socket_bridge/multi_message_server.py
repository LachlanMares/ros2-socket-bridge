import rclpy
import queue
import pickle
import zmq

from threading import Thread

from rclpy.node import Node
from std_msgs.msg import String, Float32, UInt8
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessageServerNode(Node):
    def __init__(self):
        super().__init__('multi_message_server_node')
        
        # Queue
        self.message_queue = queue.Queue(maxsize=16)

        # ZMQ
        context = zmq.Context()
        self.send_socket = context.socket(zmq.PAIR)
        self.send_socket.bind(f"tcp://*:7010")
        self.recieve_socket = context.socket(zmq.PAIR)
        self.recieve_socket.bind(f"tcp://*:7011")
        self.recieve_socket.RCVTIMEO = 10  # Milliseconds

        # Topics
        self.uint8_topic = '/uint8'
        self.float32_topic = '/float32'
        self.string_topic = "/string"
        self.odom_topic = "/odometry"
        self.pose_topic = "/pose"
        self.imu_topic = "/imu"

        # Subscribers
        self.string_subscriber = self.create_subscription(String, self.string_topic, self.string_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        # Publishers
        self.uint8_publisher = self.create_publisher(UInt8, self.uint8_topic, 10)
        self.float32_publisher = self.create_publisher(Float32, self.float32_topic, 10)

        # Timers
        self.send_loop_lock = False
        self.recieve_loop_lock = False
        self.send_loop_timer = self.create_timer(0.002, self.send_loop)
        self.recieve_loop_timer = self.create_timer(0.002, self.recieve_loop)

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

    def recieve_loop(self, ):
        if not self.recieve_loop_lock:
            self.recieve_loop_lock = True
            
            try:
                message_dict = pickle.loads(self.recieve_socket.recv())
                
                if message_dict["topic"] == self.uint8_topic:
                    string_thread = Thread(target=self.publish_uint8_msg, args=(message_dict["payload"], ))
                    string_thread.start()

                elif message_dict["topic"] == self.float32_topic:
                    odometry_thread = Thread(target=self.publish_float32_msg, args=(message_dict["payload"], ))
                    odometry_thread.start()

            except Exception as e:
                if "Resource temporarily unavailable" in str(e):  # Need to set a timeout or the script hangs on the .recv() method
                    pass

                else:
                    self.get_logger().info(str(e))

            self.recieve_loop_lock = False
       
    def publish_uint8_msg(self, msg):
        self.uint8_publisher.publish(msg) 

    def publish_float32_msg(self, msg):
        self.float32_publisher.publish(msg)

    def string_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": self.string_topic, "payload": msg}), block=False)

        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def odometry_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": self.odom_topic, "payload": msg}), block=False)

        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def pose_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": self.pose_topic, "payload": msg}), block=False)
        
        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def imu_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": self.imu_topic, "payload": msg}), block=False)
        
        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)
    

def main(args=None):
    rclpy.init(args=args)

    multi_message_server_node = MultiMessageServerNode()

    rclpy.spin(multi_message_server_node)

    multi_message_server_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
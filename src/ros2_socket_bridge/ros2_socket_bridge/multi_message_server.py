import rclpy
import queue
import pickle
import zmq

from threading import Thread

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class MultiMessageServerNode(Node):
    def __init__(self):
        super().__init__('multi_message_server_node')

        self.message_queue = queue.Queue(maxsize=16)

        context = zmq.Context()

        self.send_socket = context.socket(zmq.PAIR)
        self.send_socket.bind(f"tcp://*:7010")
        
        self.recieve_socket = context.socket(zmq.PAIR)
        self.recieve_socket.bind(f"tcp://*:7011")
        self.recieve_socket.RCVTIMEO = 5000  # Milliseconds

        self.string_subscriber = self.create_subscription(String, '/string', self.string_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

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

    def send_loop(self, ):
        while self.send_loop_running:  # Where is the equivalent to rospy.is_shutdown()?
            try:
                queue_data = self.message_queue.get(timeout=0.001)
                self.send_socket.send(queue_data)

            except queue.Empty:
                pass

            except Exception as e:
                self.get_logger().info(e)

    def recieve_loop(self, ):
        # TODO: something useful
        pass

    def string_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": "/string", "payload": msg}), block=False)

        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def odometry_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": "/odometry", "payload": msg}), block=False)

        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def pose_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": "/pose", "payload": msg}), block=False)
        
        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)

    def imu_callback(self, msg):
        try:
            self.message_queue.put(pickle.dumps({"topic": "/imu", "payload": msg}), block=False)
        
        except queue.Full:
            self.get_logger().info('Queue Full')
        
        except Exception as e:
            self.get_logger().info(e)
    

def main(args=None):
    rclpy.init(args=args)

    multi_message_server_node = MultiMessageServerNode()
    multi_message_server_node.start()

    rclpy.spin(multi_message_server_node)

    multi_message_server_node.stop()
    multi_message_server_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
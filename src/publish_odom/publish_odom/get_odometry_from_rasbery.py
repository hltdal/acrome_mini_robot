# raspberry den veri alabilmak için TCP server oluşturma
# 1. bu dosya çalıştırılacak cd raspberry && ros2 run publish_odom get_odometry_from_rasbery
# 2. windowstaki odom_tools.py adlı dosya çalıştırılacak
# 3. raspberry den veri gönderecek olan get_rpm.py dosyası çalıştırılacak (cd set_slam_ws && sudo -E python3 get_rpm.py)
# eğer motorları ilk defa çalıştıracaksan (cd set_slam_ws && sudo -E python3 first_run_motors.py) 
# komut satırında rviz2 çalıştırılacak ve fixed frame yerine kendi elinle odom yazacaksın
# ardından add tıklanacak ve odometry eklenecek
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import socket
import math
import json

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """Yaw açısını quaternion'a dönüştürür"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class OdomTCPServer(Node):
    def __init__(self):
        super().__init__('odom_tcp_server')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

        # TCP server ayarları
        HOST = "0.0.0.0"
        PORT = 5006
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((HOST, PORT))
        self.sock.listen(1)
        self.get_logger().info(f"Listening for TCP connections on port {PORT}...")

        self.conn, self.addr = self.sock.accept()
        self.get_logger().info(f"Connected by {self.addr}")

        # ROS2 timer (veri okumayı periyodik yapıyoruz)
        self.timer = self.create_timer(0.01, self.read_data)

    def read_data(self):
        try:
            self.conn.settimeout(0.001)
            data = self.conn.recv(1024)
            if not data:
                return

            odom = json.loads(data.decode())

            x = odom.get('x', 0.0)
            y = odom.get('y', 0.0)
            theta = odom.get('theta', 0.0)
            v = odom.get('v', 0.0)
            w = odom.get('w', 0.0)

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = euler_to_quaternion(theta)

            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            self.publisher_.publish(odom_msg)
            self.get_logger().info(f"Odom → x:{x:.2f}, y:{y:.2f}, θ:{theta:.2f}")

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f"Error while reading data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomTCPServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import math
import time

def euler_to_quaternion(roll, pitch, yaw):
    """Euler açılarını (rad) quaterniona çevirir"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 saniyede bir yayın

        # Örnek olarak senin elindeki veriyi burada tanımlıyoruz
        self.x = 3.658
        self.y = 2.520
        self.theta = -0.380   # rad
        self.v = 0.000
        self.w = 0.000

    def timer_callback(self):
        msg = Odometry()
        
        # Header bilgileri
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Pozisyon
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, self.theta)
        
        # Twist (hız)
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w
        
        # Kovaryanslar (örnek olarak sıfır)
        msg.pose.covariance = [0.0]*36
        msg.twist.covariance = [0.0]*36
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Odom: x={self.x:.3f}, y={self.y:.3f}, th={self.theta:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

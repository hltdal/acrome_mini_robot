import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TimeRepublisher(Node):
    def __init__(self):
        super().__init__('lidar_time_restamper')
        
        # Bu node simülasyon zamanını kullanmalı
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Gerçek Lidar'ı dinle (Örn: /scan_raw)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw', # Gerçek lidar bu topiğe basmalı
            self.listener_callback,
            qos_policy)
            
        # Simülasyon zamanıyla senkronize yeni scan yayınla
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

    def listener_callback(self, msg):
        # Mesajın zaman damgasını simülasyonun (node'un) şu anki saatiyle değiştir
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimeRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
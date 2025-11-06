import os
import socket
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanUDPSender(Node):
    def __init__(self):
        super().__init__('scan_udp_sender')

        # Target computer (automatic)
        ssh_client = os.environ.get('SSH_CLIENT')
        if ssh_client:
            self.TARGET_IP = ssh_client.split()[0]
            self.get_logger().info(f"Detected PC IP from SSH: {self.TARGET_IP}")
        else:
            self.TARGET_IP = '127.0.0.1'  # fallback
            self.get_logger().warn("SSH_CLIENT not found. Using localhost as fallback.")

        self.TARGET_PORT = 5008

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.first_message_sent = False

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info(f"Sending UDP to {self.TARGET_IP}:{self.TARGET_PORT}")

    def scan_callback(self, msg: LaserScan):
        if not self.first_message_sent:
            self.get_logger().info("First /scan message received. Starting UDP transmission...")
            self.first_message_sent = True

        try:
            scan_dict = {
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment,
                "scan_time": msg.scan_time,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": list(msg.ranges),
                "intensities": list(msg.intensities)
            }
            json_data = json.dumps(scan_dict) + '\n'
            self.sock.sendto(json_data.encode('utf-8'), (self.TARGET_IP, self.TARGET_PORT))
        except Exception as e:
            self.get_logger().warn(f"Error sending UDP data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ScanUDPSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down scan sender...")
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

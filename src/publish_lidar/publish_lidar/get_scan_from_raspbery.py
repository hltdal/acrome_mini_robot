#!/usr/bin/env python3
"""
Bu dosya WSL tarafında calisir.
1. ros2 run publish_lidar get_scan_from_raspbery komutu ile baslatilir.
2. windows tarafındaki veri TCP ile wsl e aktarılabilmesi için lidar_tools.py başlatılacak.
3. raspberry pi üzerindeki lidar verisi windows'a UDP üzerinden gönderilir. cd set_slam_ws && ros2 run lidar_pkg get_lidar
4. raspberry pi üzerinde lidar verisini /scan topic'inde yayınlar. cd lidar_ws && ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

wsl de lidar verisi /scan topic'inde LaserScan mesaji olarak yayinlar.
Rviz2 yi açıp fixed frame olarak laser_frame secilirse lidar verisi goruntulenebilir.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json
import threading
import sys

class LidarTCPServer(Node):
    def __init__(self):
        super().__init__('lidar_tcp_server')

        # LaserScan publisher
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info("'/scan' topic'i icin publisher olusturuldu.")

        # TCP sunucu mantigini ayri bir thread'de baslatarak ROS2'nin kilitlenmesini onle
        self.server_thread = threading.Thread(target=self.tcp_server_thread)
        self.server_thread.daemon = True
        self.server_thread.start()

    def tcp_server_thread(self):
        """Bu fonksiyon TCP sunucusunu calistirir, baglantilari kabul eder ve veri okur."""
        HOST = "0.0.0.0"
        PORT = 5007
        
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_sock.bind((HOST, PORT))
        except OSError as e:
            self.get_logger().error(f"!!! BIND BASARISIZ: {e}. Baska bir program {PORT} portunu kullaniyor mu?")
            rclpy.shutdown()
            sys.exit(1)
            
        server_sock.listen(1)
        self.get_logger().info(f"TCP sunucusu {PORT} portunda baglanti bekliyor...")

        while rclpy.ok():
            try:
                conn, addr = server_sock.accept()
                self.get_logger().info(f"Windows Relay'den baglanti kabul edildi: {addr}")
                
                buffer = ""
                while rclpy.ok():
                    data = conn.recv(8192)
                    if not data:
                        break  # Baglanti koptu
                    
                    buffer += data.decode('utf-8')
                    
                    # Buffer'da newline ile biten tam bir JSON mesaji var mi diye kontrol et
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line:
                            self.parse_and_publish(line)

            except (BrokenPipeError, ConnectionResetError):
                self.get_logger().warn(f"Windows Relay ile baglanti koptu. Yeni baglanti bekleniyor...")
            except Exception as e:
                self.get_logger().error(f"TCP sunucu thread'inde hata: {e}")
            finally:
                if 'conn' in locals() and conn:
                    conn.close()

    def parse_and_publish(self, json_string: str):
        """Gelen JSON string'ini ayristirir ve ROS2 topic'ine yayinlar."""
        try:
            scan_data = json.loads(json_string)

            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "laser_frame" # RViz'de goruntulemek icin onemli

            # Gelen JSON verisinden degerleri guvenli bir sekilde al
            msg.angle_min = float(scan_data.get("angle_min", 0.0))
            msg.angle_max = float(scan_data.get("angle_max", 6.2831))
            msg.angle_increment = float(scan_data.get("angle_increment", 0.0174))
            msg.time_increment = float(scan_data.get("time_increment", 0.0))
            msg.scan_time = float(scan_data.get("scan_time", 0.1))
            msg.range_min = float(scan_data.get("range_min", 0.15))
            msg.range_max = float(scan_data.get("range_max", 12.0))
            msg.ranges = [float(r) for r in scan_data.get("ranges", [])]
            msg.intensities = [float(i) for i in scan_data.get("intensities", [])]
            
            if not msg.ranges:
                self.get_logger().warn("Mesafe verisi olmayan (bos) bir Lidar paketi alindi. Yayinlanmiyor.")
                return

            self.publisher_.publish(msg)
            self.get_logger().info(f"{len(msg.ranges)} noktali LaserScan mesaji yayinlandi.", throttle_duration_sec=1.0)

        except json.JSONDecodeError:
            self.get_logger().warn(f"Gelen veri JSON formatinda degil: '{json_string[:100]}...'")
        except Exception as e:
            self.get_logger().error(f"Veri islenirken hata olustu: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTCPServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program kapatiliyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
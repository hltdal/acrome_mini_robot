#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.speed = 3  # m/s eşdeğeri, robotuna göre ayarla
        self.get_logger().info("WASD ile kontrol başlatıldı. 'q' ile çık.")

    def publish(self, left, right):
        msg = Float64MultiArray()
        msg.data = [left, right]
        self.publisher.publish(msg)
        self.get_logger().info(f"Komut gönderildi: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        while rclpy.ok():
            key = get_key()
            if key.lower() == 'w':  # ileri
                node.publish(-node.speed, -node.speed)
            elif key.lower() == 's':  # geri
                node.publish(node.speed, node.speed)
            elif key.lower() == 'a':  # sola
                node.publish(node.speed, -node.speed)
            elif key.lower() == 'd':  # sağa
                node.publish(-node.speed, node.speed)
            elif key.lower() == 'q':  # çıkış
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
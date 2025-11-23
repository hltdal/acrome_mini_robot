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
        super().__init__('teleop_rpm')
        self.publisher = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        
        # RPM değerleri
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # Artış miktarları
        self.linear_step = 10  # her basışta RPM artışı
        self.angular_step = 10  # dönme artışı
        self.max_rpm = 100      # maksimum RPM
        self.min_rpm = -100     # minimum RPM

        self.get_logger().info("Teleop başlatıldı. WASD ile RPM arttır/azalt, q ile çık.")

    def publish(self):
        msg = Float64MultiArray()
        msg.data = [self.left_rpm/60, self.right_rpm/60]  # RPM'den RPS'ye dönüştür cuz controller expects RPS
        self.publisher.publish(msg)
        self.get_logger().info(f"Left RPM: {self.left_rpm:.1f}, Right RPM: {self.right_rpm:.1f}")

    def update_rpm(self, key):
        if key.lower() == 'w':  # ileri
            self.left_rpm -= self.linear_step
            self.right_rpm -= self.linear_step
        elif key.lower() == 's':  # geri
            self.left_rpm += self.linear_step
            self.right_rpm += self.linear_step
        elif key.lower() == 'a':  # sola dön
            self.left_rpm += self.angular_step
            self.right_rpm -= self.angular_step
        elif key.lower() == 'd':  # sağa dön
            self.left_rpm -= self.angular_step
            self.right_rpm += self.angular_step
        elif key.lower() == 'x':  # fren / durdur
            self.left_rpm = 0
            self.right_rpm = 0

        # Limitleri uygula
        self.left_rpm = max(min(self.left_rpm, self.max_rpm), self.min_rpm)
        self.right_rpm = max(min(self.right_rpm, self.max_rpm), self.min_rpm)

        self.publish()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        while rclpy.ok():
            key = get_key()
            if key.lower() == 'q':
                break
            node.update_rpm(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import sys
import termios
import tty
import time
import math
import threading

# --- Kullanıcıdan Gelen Matematiksel Model ---
class DifferentialOdometry:
    def __init__(self, wheel_radius=0.036, wheel_separation=0.258):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        # Robotun durumu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def update(self, rpm_left, rpm_right):
        # Zaman farkı
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # RPM -> rad/s
        wl = (2 * math.pi / 60.0) * rpm_left
        wr = (2 * math.pi / 60.0) * rpm_right

        # Tekerlek hızları (m/s)
        vl = wl * self.wheel_radius
        vr = wr * self.wheel_radius

        # Diferansiyel sürüş denklemleri (Senin modelin: vl-vr)
        # Not: Genelde (vr-vl) kullanılır ama senin modeline sadık kalıyorum.
        # Eğer robot ters dönerse burayı (vr - vl) yapabilirsin.
        v = (vr + vl) / 2.0
        w = (vl - vr) / self.wheel_separation

        # Pozisyon güncelleme
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Theta'yı [-pi, pi] aralığında tut
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta, v, w

# --- Klavye Okuma Fonksiyonu (Bloklayıcı) ---
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- Euler Açısını Quaternion'a Çevirme ---
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Basit bir Euler -> Quaternion dönüşümü.
    Sadece 2D (Yaw) hareketi için optimize edilmiştir.
    """
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_rpm_with_odom')
        
        # Publisherlar
        self.vel_publisher = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Odometry Hesaplayıcı Sınıfı Başlat
        self.odom_calculator = DifferentialOdometry()

        # RPM değerleri
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # Artış miktarları
        self.linear_step = 10 
        self.angular_step = 10 
        self.max_rpm = 100      
        self.min_rpm = -100    

        self.running = True # Thread kontrolü için

        # Odometry Timer (Sürekli hesaplama için 20Hz - 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Teleop ve Odom Başlatıldı. WASD: Hareket, X: Dur, Q: Çıkış")

    def publish_velocity(self):
        # Tekerlek hızlarını yayınla
        msg = Float64MultiArray()
        # Controller muhtemelen rad/s (RPS) bekliyor, senin koda sadık kalarak /60 yapıyorum
        # Ama genelde RPS radyan/saniye'dir, RPM/60 ise devir/saniye'dir (Hz).
        # Senin orijinal kodunda bu dönüşüm RPM/60 (Hz) idi.
        msg.data = [self.left_rpm/60.0, self.right_rpm/60.0] 
        self.vel_publisher.publish(msg)

    def timer_callback(self):
        """
        Bu fonksiyon her 0.05 saniyede bir çağrılır.
        Klavye basılmasa bile odometry hesaplar ve yayınlar.
        """
        # 1. Odometry Hesapla
        x, y, theta, v, w = self.odom_calculator.update(self.left_rpm, self.right_rpm)

        # 2. Odometry Mesajını Oluştur
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pozisyon
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        # Yönelim (Quaternion)
        odom_msg.pose.pose.orientation = get_quaternion_from_euler(0, 0, theta)

        # Hız (Twist) - Robotun base_link referansındaki hızı
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        # 3. Yayınla
        self.odom_publisher.publish(odom_msg)

    def update_rpm(self, key):
        if key.lower() == 'w':
            self.left_rpm -= self.linear_step
            self.right_rpm -= self.linear_step
        elif key.lower() == 's':
            self.left_rpm += self.linear_step
            self.right_rpm += self.linear_step
        elif key.lower() == 'a':
            self.left_rpm += self.angular_step
            self.right_rpm -= self.angular_step
        elif key.lower() == 'd':
            self.left_rpm -= self.angular_step
            self.right_rpm += self.angular_step
        elif key.lower() == 'x':
            self.left_rpm = 0.0
            self.right_rpm = 0.0

        # Limitler
        self.left_rpm = max(min(self.left_rpm, self.max_rpm), self.min_rpm)
        self.right_rpm = max(min(self.right_rpm, self.max_rpm), self.min_rpm)

        self.publish_velocity()
        print(f"\rL_RPM: {self.left_rpm:.1f}, R_RPM: {self.right_rpm:.1f} | X: {self.odom_calculator.x:.2f} Y: {self.odom_calculator.y:.2f}", end="")

    def stop(self):
        self.running = False

# --- Thread ile Klavye Dinleme ---
def keyboard_loop(node):
    try:
        while node.running:
            key = get_key()
            if key.lower() == 'q':
                node.stop()
                rclpy.shutdown()
                break
            node.update_rpm(key)
    except Exception as e:
        print(e)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # Klavye okumayı ayrı bir thread'e alıyoruz ki main thread Odom yayınlamaya devam edebilsin
    input_thread = threading.Thread(target=keyboard_loop, args=(node,))
    input_thread.daemon = True
    input_thread.start()

    try:
        # Main thread sadece ROS callback'lerini (Timer) işler
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        input_thread.join()

if __name__ == '__main__':
    main()
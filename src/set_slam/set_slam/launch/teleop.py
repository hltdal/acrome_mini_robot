#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import sys
import termios
import tty
import math
import threading

# --- Matematiksel Model ---
class DifferentialOdometry:
    def __init__(self, wheel_radius=0.036, wheel_separation=0.258):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, rpm_left, rpm_right, dt):
        # RPM -> rad/s dönüşümü
        wl = (rpm_left * 2 * math.pi) / 60.0
        wr = (rpm_right * 2 * math.pi) / 60.0

        # Tekerlek çizgisel hızları (m/s)
        vl = wl * self.wheel_radius
        vr = wr * self.wheel_radius

        # Robotun çizgisel (v) ve açısal (w) hızı
        v = (vr + vl) / 2.0
        w = (vl - vr) / self.wheel_separation

        # --- DÜZELTME BURADA: Runge-Kutta 2 (Midpoint Integration) ---
        # 1. Bu zaman dilimindeki açı değişim miktarı
        delta_theta = w * dt

        # 2. Hareketin "tam ortasındaki" açıyı hesapla
        # Eğer sadece eski theta'yı kullanırsan (self.theta), dönüşü kaçırır.
        # Eğer yeni theta'yı kullanırsan, bu sefer de başlangıcı kaçırır.
        # En doğrusu ortalamasını almaktır.
        mid_theta = self.theta + (delta_theta / 2.0)

        # 3. Pozisyonu ORTA açıya göre güncelle (Kavisli yolu hesaba katar)
        self.x += v * math.cos(mid_theta) * dt
        self.y += v * math.sin(mid_theta) * dt

        # 4. Robotun gerçek açısını güncelle
        self.theta += delta_theta

        # Theta normalizasyonu [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta, v, w

# --- Klavye Fonksiyonları ---
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_rpm_with_odom')
        
        param = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([param])
        
        # Publisherlar
        self.vel_publisher = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        
        self.odom_calculator = DifferentialOdometry()

        # Değişkenler
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.linear_step = 10 
        self.angular_step = 10 
        self.max_rpm = 100      
        self.min_rpm = -100    
        self.running = True

        # Zaman hesabı için ROS Clock kullanımı
        self.last_time = self.get_clock().now()

        # 20Hz Timer (0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Teleop Başladı! Odom ve TF yayınlanıyor. (use_sim_time=True)")
        print("WASD: Sür, X: Dur, Q: Çıkış")

    def publish_velocity(self):
        msg = Float64MultiArray()
        # Controller muhtemelen rad/s bekliyor (RPM -> rad/s dönüşümü gerekebilir)
        # Ancak senin orijinal kodunda RPM/60 (Hz) gönderiliyordu, aynen bırakıyorum:
        msg.data = [self.left_rpm/60.0, self.right_rpm/60.0] 
        self.vel_publisher.publish(msg)

    def timer_callback(self):
        # 1. Delta Time (dt) Hesabı (ROS Saati ile)
        current_time = self.get_clock().now()
        # nanosaniye -> saniye dönüşümü
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Eğer dt çok büyükse (ilk başlangıç vb.) veya 0 ise atla
        if dt <= 0:
            return

        # 2. Odometry Hesapla
        x, y, theta, v, w = self.odom_calculator.update(self.left_rpm, self.right_rpm, dt)
        
        # Quaternion Hazırla
        q = get_quaternion_from_euler(0, 0, theta)
        current_header_stamp = current_time.to_msg()

        # --- A) Odometry Mesajı Yayını (/odom topic) ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_header_stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pozisyon
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q

        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        # Hız
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

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

        self.left_rpm = max(min(self.left_rpm, self.max_rpm), self.min_rpm)
        self.right_rpm = max(min(self.right_rpm, self.max_rpm), self.min_rpm)
        self.publish_velocity()
        
        # Print'i throttle'lamak iyi olur ama şimdilik kalsın
        print(f"\rL: {self.left_rpm:.0f}, R: {self.right_rpm:.0f} | X: {self.odom_calculator.x:.2f} Y: {self.odom_calculator.y:.2f} th: {self.odom_calculator.theta:.2f}", end="")

    def stop(self):
        self.running = False

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
    
    # Node oluştur
    node = TeleopNode()

    # Thread başlat
    input_thread = threading.Thread(target=keyboard_loop, args=(node,))
    input_thread.daemon = True
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        input_thread.join()

if __name__ == '__main__':
    main()
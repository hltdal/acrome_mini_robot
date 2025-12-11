# python3 /home/halit/acrome_mini_robot/src/set_slam/set_slam/launch/teleop.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import sys
import termios
import tty
import math
import threading
import socket
import json
import subprocess
import re

# --- OTOMATİK IP BULMA FONKSİYONU ---
def get_windows_host_ip():
    """
    WSL 2 ortamında Windows host makinesinin IP adresini bulur.
    Genellikle 'default via' rotası Windows makinesidir.
    """
    try:
        # 'ip route' komutunu çalıştır
        result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
        output = result.stdout
        
        # 'default via X.X.X.X' satırını regex ile yakala
        match = re.search(r'default via ([\d\.]+)', output)
        if match:
            ip = match.group(1)
            return ip
        
        # Alternatif: /etc/resolv.conf içindeki nameserver'a bak
        with open('/etc/resolv.conf', 'r') as f:
            for line in f:
                if line.startswith('nameserver'):
                    return line.split()[1]
                    
    except Exception as e:
        print(f"IP Bulma Hatası: {e}")
    
    return "127.0.0.1" # Bulunamazsa localhost (veya hata verebilir)

# --- Matematiksel Model ---
class DifferentialOdometry:
    def __init__(self, wheel_radius=0.036, wheel_separation=0.258):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, rpm_left, rpm_right, dt):
        wl = (rpm_left * 2 * math.pi) / (-60.0)
        wr = (rpm_right * 2 * math.pi) / (-60.0)
        vl = wl * self.wheel_radius
        vr = wr * self.wheel_radius

        v = - (vr + vl) / 2.0
        w = (vr - vl) / self.wheel_separation

        delta_theta = w * dt
        mid_theta = self.theta + (delta_theta / 2.0)

        self.x += v * math.cos(mid_theta) * dt
        self.y += v * math.sin(mid_theta) * dt
        self.theta += delta_theta
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

        # --- AĞ BAĞLANTISI ---
        self.windows_port = 5006
        self.windows_ip = get_windows_host_ip() # Otomatik IP Bul
        
        self.get_logger().info(f"Hedef Windows IP'si tespit edildi: {self.windows_ip}")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.1)
        self.connected = False
        self.connect_to_windows()

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Teleop Başladı! (Simülasyon + Gerçek Robot)")
        print("WASD: Sür, X: Dur, Q: Çıkış")

    def connect_to_windows(self):
        try:
            self.sock.connect((self.windows_ip, self.windows_port))
            self.connected = True
            self.get_logger().info(f"Windows Bridge'e bağlandı: {self.windows_ip}:{self.windows_port}")
        except Exception as e:
            # Hata mesajını sürekli basmamak için debug seviyesinde veya sadece ilk seferde uyarabiliriz
            # Ama bağlantı yoksa simülasyon devam etmeli
            pass
            self.connected = False

    def send_to_windows(self):
        if not self.connected:
            # Bağlantı yoksa tekrar dene (ama çok sık değil, burada basit bir logic var)
            # Normalde bunu timer içinde yapmak daha safe ama bu da çalışır.
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(0.01) # Hızlı fail olsun
                self.connect_to_windows()
            except:
                pass
            return

        data = {
            "L": round(self.left_rpm, 2),
            "R": round(self.right_rpm, 2)
        }
        json_data = json.dumps(data) + "\n"

        try:
            self.sock.sendall(json_data.encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("Windows bağlantısı koptu!")
            self.connected = False
            self.sock.close()

    def publish_velocity(self):
        # 1. Simülasyon
        msg = Float64MultiArray()
        msg.data = [self.left_rpm/(7.5), self.right_rpm/(7.5)] 
        self.vel_publisher.publish(msg)

        # 2. Gerçek Robot (Windows Relay)
        self.send_to_windows()

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0: return

        x, y, theta, v, w = self.odom_calculator.update(self.left_rpm, self.right_rpm, dt)
        
        q = get_quaternion_from_euler(0, 0, theta)
        current_header_stamp = current_time.to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = current_header_stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q

        # EKF için Kovaryans Değerleri (GÜVEN MATRİSİ)
        # Çapraz elemanlar: x, y, z, roll, pitch, yaw
        # Tekerlek odometrisi X ve Yaw konusunda iyidir ama kayabilir.
        # Değer ne kadar küçükse, EKF ona o kadar güvenir.
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # X hatası (düşük)
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # Y hatası
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Roll
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1    # Yaw hatası (dönüşte kayma olabilir)
        ]
        
        # Hız kovaryansı (Twist)
        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # Vx
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # Vy
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1    # Vyaw
        ]

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
        
        print(f"\rL: {self.left_rpm:.0f}, R: {self.right_rpm:.0f} | X: {self.odom_calculator.x:.2f} Y: {self.odom_calculator.y:.2f}", end="")

    def stop(self):
        self.running = False
        if self.connected:
            self.sock.close()

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
    input_thread = threading.Thread(target=keyboard_loop, args=(node,))
    input_thread.daemon = True
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        input_thread.join()

if __name__ == '__main__':
    main()
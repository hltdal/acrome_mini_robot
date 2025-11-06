#sudo -E python3 get_rpm.py

from smd.red import *  # Import the SMD Red motor control library
from serial.tools.list_ports import comports  # Import serial communication tools
from platform import system  # Import system information module
import os
import time
import threading
from set_odometry import DifferentialOdometry  # Import odometry class
import socket
import json

# --- Hedef IP'yi Otomatik Algılama ---
ssh_client = os.environ.get('SSH_CLIENT')
if ssh_client:
    # ssh_client '192.168.1.158 54321 22' gibi bir string içerir.
    # Bize sadece ilk kısım (IP adresi) lazım.
    UDP_IP = ssh_client.split()[0]
    print(f"PC IP'si SSH üzerinden otomatik algılandı: {UDP_IP}")
else:
    UDP_IP = '127.0.0.1'  # Eğer SSH ile bağlı değilse (test vb.)
    print("UYARI: SSH_CLIENT ortam değişkeni bulunamadı.")
    print("Fallback olarak localhost (127.0.0.1) kullanılıyor.")

UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Nereye gönderim yaptığını teyit etmek için bir print ekleyelim
print(f"UDP paketleri şu hedefe gönderilecek: {UDP_IP}:{UDP_PORT}")


# Function to detect and return the correct USB port for communication
def USB_Port():
    ports = list(comports())
    usb_names = {
        "Windows": ["USB Serial Port"],
        "Linux": ["/dev/ttyUSB"],
        "Darwin": [
            "/dev/tty.usbserial",
            "/dev/tty.usbmodem",
            "/dev/tty.SLAB_USBtoUART",
            "/dev/tty.wchusbserial",
            "/dev/cu.usbserial",
            "/dev/cu.usbmodem",
            "/dev/cu.SLAB_USBtoUART",
            "/dev/cu.wchusbserial",
        ]
    }

    os_name = system()
    if ports:
        for port, desc, hwid in sorted(ports):
            if any(name in port or name in desc for name in usb_names.get(os_name, [])):
                return port
        print("Current ports:")
        for port, desc, hwid in ports:
            print(f"Port: {port}, Description: {desc}, Hardware ID: {hwid}")
    else:
        print("No port found")
    return None


# --- Motor ayarları ---
port = USB_Port()


m = Master(port)

ID1 = 0
ID2 = 1

m.attach(Red(ID1))
m.attach(Red(ID2))

m.set_shaft_cpr(ID1, 6533)
m.set_shaft_rpm(ID1, 100)
m.set_operation_mode(ID1, OperationMode.Velocity)
m.set_control_parameters_velocity(ID1, 30.0, 5.0, 0.0)
m.enable_torque(ID1, True)

m.set_shaft_cpr(ID2, 6533)
m.set_shaft_rpm(ID2, 100)
m.set_operation_mode(ID2, OperationMode.Velocity)
m.set_control_parameters_velocity(ID2, 30.0, 5.0, 0.0)
m.enable_torque(ID2, True)


# --- Odometri ---
odo = DifferentialOdometry()


def odometry_loop():
    """Arka planda sürekli odometri verisi yayınlayan thread."""
    while True:
        rpm_left = m.get_velocity(ID1)
        rpm_right = m.get_velocity(ID2)

        x, y, theta, v, w = odo.update(-rpm_left, rpm_right)

        print("\n--- ODOMETRY ---")
        print(f"Left RPM: {rpm_left:.2f}, Right RPM: {rpm_right:.2f}")
        print(f"x={x:.3f}, y={y:.3f}, theta={theta:.3f}, v={v:.3f}, w={w:.3f}")
        print("----------------\n")

        # --- UDP ile gönder ---
        odom_data = {
            "x": x, "y": y, "theta": theta,
            "v": v, "w": w,
            "rpm_left":rpm_left, "rpm_right":rpm_right,
            "timestamp": time.time()
        }
        message = json.dumps(odom_data).encode("utf-8")
        sock.sendto(message, (UDP_IP, UDP_PORT))
        print("odom's data is sended")

        time.sleep(1)  # 1 saniye aralıklarla güncelle


# Thread başlat
thread = threading.Thread(target=odometry_loop, daemon=True)
thread.start()


# --- Kullanıcı inputları (main thread) ---
while True:
    try:
        speed1 = float(input("Motor 1 Speed (RPM): "))
        m.set_velocity(ID1, speed1)

        speed2 = float(input("Motor 2 Speed (RPM): "))
        m.set_velocity(ID2, speed2)

    except ValueError:
        print("pls enter valid number")
    except KeyboardInterrupt:
        print("Çıkılıyor...")
        break
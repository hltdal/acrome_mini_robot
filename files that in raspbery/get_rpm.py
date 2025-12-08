# sudo -E python3 get_rpm.py

from smd.red import * # Motor kütüphanesi
from serial.tools.list_ports import comports
from platform import system
import socket
import json
import os

# --- AĞ AYARLARI (Sadece Dinleme) ---
LISTEN_IP = "0.0.0.0"  # Tüm ağ arayüzlerinden dinle
LISTEN_PORT = 5005     # Windows'un gönderdiği port

# --- SOKET KURULUMU ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
sock.setblocking(True) # Veri gelene kadar bekle

print(f"UDP Sunucusu Başlatıldı.")
print(f"DİNLİYOR: {LISTEN_IP}:{LISTEN_PORT}")
print("Görevi: Sadece gelen motor komutlarını uygular.")

# --- USB PORT TESPİTİ ---
def USB_Port():
    ports = list(comports())
    usb_names = {
        "Windows": ["USB Serial Port"],
        "Linux": ["/dev/ttyUSB"],
        "Darwin": ["/dev/tty.usbserial"]
    }
    os_name = system()
    if ports:
        for port, desc, hwid in sorted(ports):
            if any(name in port or name in desc for name in usb_names.get(os_name, [])):
                return port
    return None

# --- MOTOR SÜRÜCÜ AYARLARI ---
port = USB_Port()
if port is None:
    print("HATA: Motor sürücü USB portu bulunamadı! Bağlantıyı kontrol edin.")
    exit(1)

m = Master(port)

ID1 = 0 # Sol Motor
ID2 = 1 # Sağ Motor

# Motorları sisteme tanıt
m.attach(Red(ID1))
m.attach(Red(ID2))

# Motor Parametreleri (Senin verdiğin değerler)
# Motor 1
m.set_shaft_cpr(ID1, 6533)
m.set_shaft_rpm(ID1, 100)
m.set_operation_mode(ID1, OperationMode.Velocity)
m.set_control_parameters_velocity(ID1, 30.0, 5.0, 0.0)
m.enable_torque(ID1, True)

# Motor 2
m.set_shaft_cpr(ID2, 6533)
m.set_shaft_rpm(ID2, 100)
m.set_operation_mode(ID2, OperationMode.Velocity)
m.set_control_parameters_velocity(ID2, 30.0, 5.0, 0.0)
m.enable_torque(ID2, True)

print("Motorlar aktif ve tork açık. Veri bekleniyor...")

# --- ANA DÖNGÜ ---
try:
    while True:
        # 1. Veri Bekle (Bloklayıcı)
        data, addr = sock.recvfrom(1024)
        
        if not data:
            continue

        try:
            # 2. Gelen Veriyi Çöz (JSON Parsing)
            decoded_data = data.decode('utf-8').strip()
            command = json.loads(decoded_data)
            
            # Beklenen format: {"L": 50, "R": 50}
            if "L" in command and "R" in command:
                target_l = float(command["L"])
                target_r = float(command["R"])
                
                # 3. Motorlara Hızı Uygula
                m.set_velocity(ID1, target_l)
                m.set_velocity(ID2, target_r)
                
                # Debug (İstersen açabilirsin, çok hızlı veri gelirse terminali doldurur)
                # print(f"Hız Ayarlandı -> L: {target_l}, R: {target_r}")
                
            else:
                print(f"Eksik Veri: {command}")

        except json.JSONDecodeError:
            print(f"Hatalı JSON Paketi: {decoded_data}")
        except Exception as e:
            print(f"Motor Hatası: {e}")

except KeyboardInterrupt:
    print("\nProgram durduruluyor...")
    # Güvenli çıkış için motorları durdur
    m.set_velocity(ID1, 0)
    m.set_velocity(ID2, 0)
    sock.close()
    print("Motorlar durduruldu, soket kapatıldı.")
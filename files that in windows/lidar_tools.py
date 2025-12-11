import socket
import time

# Raspberry’den gelen UDP verisi
UDP_IP = "0.0.0.0"
UDP_PORT = 5008

# WSL hedefi
TARGET_IP = "localhost"
TARGET_PORT = 5007

def connect_to_wsl():
    while True:
        try:
            print(f"Connecting to WSL: {TARGET_IP}:{TARGET_PORT}")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((TARGET_IP, TARGET_PORT))
            print("Connection to WSL successful!")
            return sock
        except Exception as e:
            print(f"Connection error: {e}, retrying in 5 seconds...")
            time.sleep(5)

def main():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP started. Raspberry is listening on {UDP_PORT}...")

    tcp_sock = connect_to_wsl()

    while True:
        try:
            data, addr = udp_sock.recvfrom(65535)  # Lidar verisi büyük olabilir
            if data:
                tcp_sock.sendall(data)
        except (BrokenPipeError, ConnectionResetError):
            print("WSL bağlantısı koptu. Yeniden bağlanılıyor...")
            tcp_sock.close()
            tcp_sock = connect_to_wsl()
        except Exception as e:
            print(f"Hata: {e}")
            time.sleep(1)

if __name__ == '__main__':
    main()

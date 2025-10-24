import socket, json

# Raspberry'den gelecek odometry
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind((UDP_IP, UDP_PORT))

# WSL TCP server bilgisi
TCP_IP = "localhost"  # WSL2’de Windows’tan erişim için localhost değil, WSL IP olmalı. Genelde: 172.20.x.x
TCP_PORT = 5006
sock_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_send.connect((TCP_IP, TCP_PORT))

print("Listening for odometry data...")

while True:
    data, addr = sock_recv.recvfrom(1024)
    odom = json.loads(data.decode("utf-8"))
    print(f"Received from {addr}: {odom}")
    odom_data = {
        "x": odom["x"], "y": odom["y"], "theta": odom["theta"],
        "v": odom["v"], "w": odom["w"],
        "rpm_left": odom["rpm_left"], "rpm_right": odom["rpm_right"],
        "timestamp": odom["timestamp"]
    }
    message = json.dumps(odom_data)
    sock_send.send(message.encode("utf-8"))

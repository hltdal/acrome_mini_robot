import socket
import json

# --- AYARLAR ---
WSL_LISTEN_IP = "0.0.0.0"   # Tüm arayüzlerden dinle
WSL_LISTEN_PORT = 5006      # WSL'den veriyi alacağımız port

# Raspberry Pi IP Adresi (Kendi Pi IP'n ile değiştir)
RASPBERRY_IP = "192.168.1.153"
RASPBERRY_PORT = 5005       # Pi'ye göndereceğimiz port

def start_relay():
    # 1. WSL'den dinlemek için TCP Sunucusu
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind((WSL_LISTEN_IP, WSL_LISTEN_PORT))
    server_sock.listen(1)
    
    # 2. Raspberry Pi'ye göndermek için UDP Soketi
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Relay Başlatıldı!")
    print(f"DİNLİYOR (WSL'den): Port {WSL_LISTEN_PORT}")
    print(f"HEDEF (Raspberry Pi): {RASPBERRY_IP}:{RASPBERRY_PORT}")
    
    while True:
        print("\nWSL'den bağlantı bekleniyor...")
        conn, addr = server_sock.accept()
        print(f"Bağlandı: {addr}")
        
        try:
            buffer = ""
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                
                # Gelen veriyi buffer'a ekle
                buffer += data.decode('utf-8')
                
                # Satır satır işle (JSON mesajları \n ile ayrılır)
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip(): continue
                    
                    try:
                        # Gelen veri: {"L": 50, "R": 50}
                        # JSON kontrolü (sadece geçerli veriyi gönderelim)
                        json_obj = json.loads(line)
                        
                        # Raspberry Pi'ye Ham JSON gönder
                        msg_bytes = line.encode('utf-8')
                        udp_sock.sendto(msg_bytes, (RASPBERRY_IP, RASPBERRY_PORT))
                        
                        print(f"\rİletildi -> Pi: {line}", end="")
                        
                    except json.JSONDecodeError:
                        print(f"Hatalı JSON: {line}")
                        
        except Exception as e:
            print(f"Bağlantı hatası: {e}")
        finally:
            conn.close()

if __name__ == "__main__":
    start_relay()
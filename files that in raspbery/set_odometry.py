import time
import math

class DifferentialOdometry:
    def __init__(self, wheel_radius=0.036, wheel_separation=0.258):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        # Robot pozisyonu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def update(self, rpm_left, rpm_right):
        # 🔄 Motor yön ve taraf düzeltmesi
        # Sol ve sağ motor yer değiştiriyor + yönler ters
        rpm_left_corrected = -rpm_right
        rpm_right_corrected = -rpm_left

        # Zaman farkı
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # RPM -> rad/s
        wl = (2 * math.pi / 60.0) * rpm_left_corrected
        wr = (2 * math.pi / 60.0) * rpm_right_corrected

        # Doğrusal hızlar (m/s)
        vl = wl * self.wheel_radius
        vr = wr * self.wheel_radius

        # Diferansiyel sürüş modeli
        v = (vr + vl) / 2.0
        w = (vr - vl) / self.wheel_separation

        # Pozisyon güncelleme
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Theta'yı [-pi, pi] aralığında tut
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta, v, w


# 🔧 Test
if __name__ == "__main__":
    odo = DifferentialOdometry()

    test_data = [
        (20, 20),     # Gerçekte sola dönüyor
        (20, -20),    # Gerçekte geri gidiyor
        (-20, 20),    # Gerçekte ileri gidiyor
        (-20, -20),   # Gerçekte sağa dönüyor
    ]

    for rpm_left, rpm_right in test_data:
        print(f"\nSol={rpm_left}, Sağ={rpm_right}")
        for _ in range(5):
            x, y, theta, v, w = odo.update(rpm_left, rpm_right)
            print(f"x={x:.3f}, y={y:.3f}, θ={theta:.3f}, v={v:.3f}, w={w:.3f}")
            time.sleep(0.1)

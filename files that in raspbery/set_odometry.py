import time
import math

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

        # Diferansiyel sürüş denklemleri
        v = (vr + vl) / 2.0
        w = (vl - vr) / self.wheel_separation

        # Pozisyon güncelleme
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Theta'yı [-pi, pi] aralığında tut
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta, v, w

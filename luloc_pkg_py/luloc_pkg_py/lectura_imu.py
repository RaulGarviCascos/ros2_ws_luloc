import smbus2
import time

class MPU6050:
    def __init__(self, address=0x68):
        self.bus = smbus2.SMBus(1)
        self.address = address
        self._setup()

    def _setup(self):
        # Despertar sensor
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        time.sleep(0.1)

    def leer_eje(self, reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg + 1)
        res = (h << 8) | l
        return res if res < 32768 else res - 65536

    def obtener_aceleracion(self):
        # Retorna valores en Gs (9.81 m/s2)
        return {
            'x': self.leer_eje(0x3B) / 16384.0,
            'y': self.leer_eje(0x3D) / 16384.0,
            'z': self.leer_eje(0x3F) / 16384.0
        }

# Ejemplo de uso:
if __name__ == "__main__":
    imu = MPU6050()
    while True:
        data = imu.obtener_aceleracion()
        print(f"X: {data['x']:.2f} G | Y: {data['y']:.2f} G | Z: {data['z']:.2f} G")
        time.sleep(0.1)

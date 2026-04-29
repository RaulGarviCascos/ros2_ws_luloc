import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time

class Mpu6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # Parámetros e I2C
        self.address = 0x68
        try:
            self.bus = smbus2.SMBus(1)
            self.iniciar_sensor()
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir el bus I2C: {e}")

        # Publicador
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Timer: 20Hz (cada 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Nodo MPU6050 iniciado correctamente")

    def iniciar_sensor(self):
        try:
            # Reset total
            self.bus.write_byte_data(self.address, 0x6B, 0x80)
            time.sleep(0.2)
            # Despertar
            self.bus.write_byte_data(self.address, 0x6B, 0x01)
            # Configurar Acelerómetro (+/- 2g)
            self.bus.write_byte_data(self.address, 0x1C, 0x00)
            self.get_logger().info("Hardware del sensor inicializado.")
        except Exception as e:
            self.get_logger().error(f"Fallo al configurar hardware: {e}")

    def leer_raw(self, reg):
        try:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
            val = (high << 8) | low
            return val if val < 32768 else val - 65536
        except OSError:
            return None

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Leer valores
        ax = self.leer_raw(0x3B)
        ay = self.leer_raw(0x3D)
        az = self.leer_raw(0x3F)

        if ax is not None:
            # Conversión a m/s² (Para +/- 2g, el factor es 16384.0)
            # Multiplicamos por 9.81 para pasar de 'g' a m/s²
            msg.linear_acceleration.x = (ax / 16384.0) * 9.81
            msg.linear_acceleration.y = (ay / 16384.0) * 9.81
            msg.linear_acceleration.z = (az / 16384.0) * 9.81

            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Error de lectura I2C - Reintentando...")

def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

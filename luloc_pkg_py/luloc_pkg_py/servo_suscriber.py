import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo, Button

# Inicialización de hardware
servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
sensor_inf = Button(17, pull_up=False) 
sensor_sup = Button(27, pull_up=False)

servo.detach()

class ROS2ServoSubscriber(Node):
    def __init__(self):
        super().__init__("servo_suscriber")
        
        # Variable interna para guardar la última velocidad solicitada
        self.target_vel = 0.0
        
        # 1. Suscriptor: Solo escucha y guarda el valor
        self.subscription_ = self.create_subscription(
            Float32,
            "robot/servo_vel",
            self.calback_servo,
            10
        )
        
        self.timer_period = 0.005  
        self.security_timer = self.create_timer(self.timer_period, self.check_security_loop)
        
        self.get_logger().info("Nodo iniciado: Suscriptor y bucle de seguridad a 50Hz activos.")

    def calback_servo(self, msg: Float32):
        self.target_vel = float(msg.data)

    def check_security_loop(self):
        puls_inf = sensor_inf.is_pressed
        puls_sup = not sensor_sup.is_pressed
        
        if self.target_vel < 0:  
            if puls_inf:
                servo.value = self.target_vel
            else:
                self.get_logger().warn("¡EMERGENCIA! Final de carrera INFERIOR no activado. Parando.", throttle_duration_sec=1.0)
                servo.detach()
                self.target_vel = 0.0  
                
        elif self.target_vel > 0:  
            if not puls_sup:
                servo.value = self.target_vel
            else:
                self.get_logger().warn("¡EMERGENCIA! Final de carrera SUPERIOR activado. Parando.", throttle_duration_sec=1.0)
                servo.detach()
                self.target_vel = 0.0
                
        else:
            servo.detach()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ServoSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
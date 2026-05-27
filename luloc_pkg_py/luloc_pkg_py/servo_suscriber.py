import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'
from gpiozero import Servo, Button
from time import sleep, time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
sensor_inf = Button(17, pull_up=False) 
sensor_sup = Button(27, pull_up=False)

servo.detach()

vel = 0.3

class ROS2ServoSubscriber(Node):
    def __init__(self):
        super().__init__("servo_suscriber")
        self.subscription_ = self.create_subscription(
            Float32,
            "cmd_vel",
            self.calback_servo,
            10
        )
        self.get_logger().info("Listener servo has been started")

    def calback_servo(self, msg: Float32):
        puls_inf = sensor_inf.is_pressed
        puls_sup = not sensor_sup.is_pressed
        vel = float(msg.data)
        if not puls_inf or not puls_sup:
            if vel == 0:
                servo.detach()
            servo.value = vel
            self.get_logger().info(f"Muevo servo con vel {vel}")
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
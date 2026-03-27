import os
import json
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_TOPIC_TELEOP = os.getenv("MQTT_TOPIC_TELEOP", "robot/teleop")

# Distancia entre centros de ruedas, en metros
L = 0.170

try:
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="cmd_vel_2_mqtt")
except AttributeError:
    mqtt_client = mqtt.Client(client_id="cmd_vel_2_mqtt")

mqtt_client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=30)
mqtt_client.loop_start()


class CmdVelToMqtt(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_mqtt")
        self.subscription_ = self.create_subscription(
            Twist,
            "cmd_vel",
            self.callback_cmd_vel,
            10
        )
        self.get_logger().info("Listener cmd_vel has been started")

    def callback_cmd_vel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        left = v - (w * L / 2.0)
        right = v + (w * L / 2.0)

        payload = {
            "l": round(left, 3),
            "r": round(right, 3)
        }

        payload_str = json.dumps(payload)
        print(payload_str)

        mqtt_client.publish(
            MQTT_TOPIC_TELEOP,
            payload_str,
            qos=0,
            retain=False,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMqtt()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
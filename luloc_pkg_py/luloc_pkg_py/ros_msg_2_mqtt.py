import os
import json
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_TOPIC_COMMAND = os.getenv("MQTT_TOPIC_COMMAND", "robot/api/request")


try:
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ros_msg_2_mqtt")
except AttributeError:
    mqtt_client = mqtt.Client(client_id="ros_msg_2_mqtt")

mqtt_client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=30)
mqtt_client.loop_start()


class ROStoMQTT(Node):
    def __init__(self):
        super().__init__("ros_msg_2_mqtt")
        self.subscription_ = self.create_subscription(
            String,
            "command",
            self.callback_command,
            10
        )
        self.get_logger().info("Listener command has been started")

    def callback_command(self, msg: String):
        str_data = msg.data
        print(f"Received ROS message: {str_data}")

        mqtt_client.publish(
            MQTT_TOPIC_COMMAND,
            str_data,
            qos=0,
            retain=False,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ROStoMQTT()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class ListenerNode(Node): 
    def __init__(self):
        super().__init__("my_listener") 
        self.subscription_ = self.create_subscription(String,"robot_news",self.callback_robot_news,10)
        self.get_logger().info("Listener has been started")
    
    def callback_robot_news(self,msg:String):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

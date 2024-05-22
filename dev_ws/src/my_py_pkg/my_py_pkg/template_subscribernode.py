#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class template_subscribernode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("template_subscribernode") # MODIFY NAME

        self.subscriber_ = self.create_subscription(String, "topicname", self.callback_news, 10) # 10 is the queue size.
        self.get_logger().info("baslangicta info olarak ne yazmak istiyorsan yaz.")

    def callback_news(self,msg):
        
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = template_subscribernode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

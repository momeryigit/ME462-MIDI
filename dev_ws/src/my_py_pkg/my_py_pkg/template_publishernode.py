#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class template_publishernode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("template_publishernode") # MODIFY NAME

        self.publisher_ = self.create_publisher(String, "topicname", 10) # 10 is the queue size.
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("baslangicta info olarak ne yazmak istiyorsan yaz.")

    def publish_news(self):
        msg = String()
        msg.data = "helooo"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = template_publishernode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

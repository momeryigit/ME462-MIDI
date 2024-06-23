import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from midibot_py import SocketCommunication as Socket


class SocketNode(Node):
    """
    A node that handles the communication between the MCU and the ROS2 network using a socket connection.
    Subscribes to the 'mcu_outgoing' topic and sends the message to the MCU.
    Publishes the message received from the MCU to the 'mcu_incoming' topic.
    """

    def __init__(self):
        super().__init__("socket_node")

        # Declare parameters with default values
        self.declare_parameter("ip", "192.168.0.24")
        self.declare_parameter("port", 8081)

        # Get parameters
        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.get_logger().info(f"IP: {ip}, Port: {port}")

        # Initialize socket client
        self.client = Socket(ip=ip, socket_port=port)

        # Try to connect to the server up to 5 times
        for _ in range(5):
            try:
                self.client.connect()
                break
            except Exception as e:
                self.get_logger().error(f"Failed to connect to server. {str(e)}")
        else:
            self.get_logger().error("Failed to connect to server after 5 attempts.")
            return

        # Set the data callback to publish the data to the 'mcu_incoming' topic
        self.client.set_data_callback(self.publish_socket_data)

        # Create a subscription to the 'mcu_outgoing' topic
        self.subscription = self.create_subscription(
            String, "mcu_outgoing", self.write_to_socket, 10
        )
        self.subscription  # Prevent unused variable warning

        # Create a publisher to the 'mcu_incoming' topic
        self.publisher = self.create_publisher(String, "mcu_incoming", 10)
        self.publisher  # Prevent unused variable warning

    def write_to_socket(self, msg):
        """
        Send the message to the socket.
        """
        self.client.send_command(msg.data)
        self.get_logger().info(f"Sent: {msg.data}")

    def publish_socket_data(self, data):
        """
        Publish the data received from the socket to the 'mcu_incoming' topic.
        """
        if data:
            msg = String(data=data)
            self.get_logger().info(f"Received: {data}")
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    socket_node = SocketNode()

    rclpy.spin(socket_node)
    socket_node.client.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    socket_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

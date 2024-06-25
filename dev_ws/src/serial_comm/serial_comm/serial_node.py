import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from midibot_py import DifferentialDriveRobot as Robot


class SerialNode(Node):
    """
    A node that handles communication between the MCU and the ROS2 network using a serial connection.
    Subscribes to the 'mcu_outgoing' topic and sends the message to the MCU via the serial port.
    Publishes messages received from the MCU to the 'mcu_incoming' topic.
    """

    def __init__(self):
        super().__init__("serial_node")

        # Declare parameters with default values
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)

        # Get parameters
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.get_logger().info(f"Port: {port}, Baudrate: {baud}")
        

        #Robot Init
        self.robot = Robot(serial_port=port, baudrate=baud)
        
        # Try connecting to robot
        try:
            self.robot.connect(connection_type="serial")
            self.get_logger().info(f"Robot connected to {port} at {baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Robot connection error: {str(e)}")

        # Create a subscription to the 'mcu_outgoing' topic
        self.subscription = self.create_subscription(
            String, "mcu_outgoing", self.write_to_serial, 10
        )
        self.subscription  # Prevent unused variable warning
        
        self.publisher = self.create_publisher(String, "mcu_incoming", 10)
        self.publisher
        
        # Set the data callback to publish the data to the 'mcu_incoming' topic
        self.robot.set_data_callback(self.publish_serial_data)

        self.get_logger().info("Successfully initialized serial node.")

    def write_to_serial(self, msg):
        """
        Send the message to the serial port.
        """
        if msg:
            self.get_logger().info(f"Sending: {msg.data}")
            self.robot.send_command(msg.data)

    def publish_serial_data(self, data):
        """
        Publish the data received from the serial port to the 'mcu_incoming' topic.
        """
        if data:
            msg = String(data=data)
            self.get_logger().info(f"Received: {data}")
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

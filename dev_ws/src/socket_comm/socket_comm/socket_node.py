import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .submodules.socket import ESP32Client
import threading



class SocketNode(Node):
    '''
    A node that handles the communication between the MCU and the ROS2 network using a socket connection.
    Subscribes to mcu_outgoing topic and sends the message to the MCU.
    Publishes the message recieved from the MCU to mcu_incoming topic.
    '''

    def __init__(self):
        super().__init__('socket_node')
        #Declare parameters
        self.declare_parameter('ip', '192.168.212.127')
        self.declare_parameter('port', 8082)
        
        #Get parameters
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.get_logger().info(f'IP: {ip}, Port: {port}')

        #Socket Initialization
        self.client = ESP32Client(ip, port)

        #try to connect to the server 5 times
        for i in range(5):
            try:
                self.client.connect()
                break
            except Exception as e:
                self.get_logger().error(f'Failed to connect to server. {str(e)}')
        else:
            self.get_logger().error('Failed to connect to server after 5 attempts.')
            return
        
        self.subscription = self.create_subscription(
            String,
            'mcu_outgoing',
            self.write_to_socket,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            String,
            'mcu_incoming',
            10)
        self.publisher  # prevent unused variable warning
        self.read_thread = threading.Thread(target=self.read_from_socket)
        self.read_thread.start()
        


    def write_to_socket(self, msg):
        self.client.send_data(msg.data)
        self.get_logger().info(f'Sent: {msg.data}')
    
    def read_from_socket(self):
        while True:
            try:
                data = self.client.recieve_newline()
            except Exception as e:
                self.get_logger().error(f'Failed to read from socket. {str(e)}')
            if data:
                self.get_logger().info(f'Received: {data}')
                self.publisher.publish(String(data=data))
    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = SocketNode()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.client.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
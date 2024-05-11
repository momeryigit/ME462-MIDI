import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .submodules.serial import Serial
import threading



class SerialNode(Node):
    '''
    A subscriber that subscribes to serial_outgoing topic and sends the message to the serial port. 
    '''

    def __init__(self):
        super().__init__('serial_node')

        #Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        
        #Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info(f'Port: {port}, Baudrate: {baud}')

        #Serial Initialization
        #try to initialize serial port 3 times
        for i in range(3):
            try:
                self.ser = Serial(port, baud)
                break
            except Exception as e:
                self.get_logger().error(f'Failed to initialize serial port. {str(e)}')
        else:
            self.get_logger().error('Failed to initialize serial port after 3 attempts.')
            return
        
        self.subscription = self.create_subscription(
            String,
            'mcu_outgoing',
            self.write_to_serial,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            String,
            'mcu_incoming',
            10)
        self.publisher  # prevent unused variable warning
        #self.writing = False

        #whenever a message other than none is recieved from serial port, it is published to mcu_incoming topic
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.start()
        self.get_logger().info('Succesfully initialized serial node.')

    def write_to_serial(self, msg):
        #self.writing = True
        self.ser.write(msg.data)
        #self.writing = False
        self.get_logger().info(f'Sent: {msg.data}')
        
    
    def read_from_serial(self):
        while True:
            msg = None
            try:
                msg = self.ser.readline().decode('utf-8')
            except Exception as e:
                self.get_logger().error(f'Failed to read from serial port. {str(e)}')
            if msg:
                self.get_logger().info(f'Received: {msg}')
                self.publisher.publish(String(data=msg))
    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = SerialNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
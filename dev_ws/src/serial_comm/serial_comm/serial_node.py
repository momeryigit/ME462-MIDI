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
        #Serial Initialization
        self.ser = Serial()
        self.subscription = self.create_subscription(
            String,
            'serial_outgoing',
            self.write_to_serial,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            String,
            'serial_incoming',
            10)
        self.publisher  # prevent unused variable warning
        self.writing = False
        #whenever a message other than none is recieved from serial port, it is published to serial_incoming topic
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.start()
        print('Succesfully initialized serial node.')

    def write_to_serial(self, msg):
        self.writing = True
        self.ser.write(msg.data)
        self.writing = False
        self.get_logger().info(f'Sent: {msg.data}')
        
    
    def read_from_serial(self):
        while self.writing == False:
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
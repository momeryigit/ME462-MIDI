import rclpy
from rclpy.node import Node
from .submodules.serialcom import SerialCom
from std_msgs.msg import String



class MinimalSubscriber(Node):
    '''
    Creates a subscriber to subscribe to cmd_vel topic. 
    '''

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'wasd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscribed to wasd topic')
        self.serial = SerialCom('/dev/ttyUSB0', 9600)

    def listener_callback(self, msg):
        self.get_logger().info(msg.data + '\n') 
        self.serial.send(msg.data)
        


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

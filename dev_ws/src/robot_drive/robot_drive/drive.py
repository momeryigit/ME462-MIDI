import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist



class DriveNode(Node):
    '''
    A drive node for 2 stepper motors.
    Inputs can be given in several ways:
    1. Linear and angular velocity in Twist message.
    2. Right and left motor ticks in String message.
    3. Right and left motor speed in String message.
    4. Right and left motor distance in String message.

    The node will have subscribers for all the above inputs. All data will be converted to ticks and frequency and will be sent to mcu_outgoing topic.
    '''

    def __init__(self):
        self.b = 0.221 #Distance between the wheels
        self.r = 0.05 #Radius of the wheel
        self.ticks_per_rev = 600 #Number of ticks per revolution of the motor
        
        super().__init__('drive_node')

        self.publisher = self.create_publisher(
            String,
            'mcu_outgoing',
            10)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.convert_twist_to_tickfreq,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher  # prevent unused variable warning

    def convert_twist_to_tickfreq(self, msg):
        '''
        This function converts the twist message to right and left wheel tick frequency
        '''
        lin_v = msg.linear.x
        ang_w = msg.angular.z
        #Calculate right and left wheel angular velocity
        w_l = (lin_v - ang_w*self.b/2)/self.r
        w_r = (lin_v + ang_w*self.b/2)/self.r
        #Calculate right and left wheel tick frequency
        f_l = w_l/(2*math.pi)*self.ticks_per_rev
        f_r = w_r/(2*math.pi)*self.ticks_per_rev
        #Reduce the frequency to 3 decimal places
        f_l = round(f_l,3)
        f_r = round(f_r,3)
        #Create string message
        msg1 = String()
        msg1.data = f's r {math.copysign(1,f_r)} {abs(f_r)}'
        msg2 = String()
        msg2.data = f's l {math.copysign(1,f_l)} {abs(f_l)}'
        #Publish the message
        self.publisher.publish(msg1)
        self.publisher.publish(msg2)


    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DriveNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
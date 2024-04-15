import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.clock import ROSClock


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

        #Robot position
        self.x = 0.0
        self.y = 0.0
        self.v_r = 0.0
        self.v_l = 0.0
        self.ang_w = 0.0
        self.theta = 0.0
        self.r_icc = 0.0
        #Robot parameters
        self.b = 0.295 #Distance between the wheels
        self.r = 0.085/2 #Radius of the wheel
        self.ticks_per_rev = 600 #Number of ticks per revolution of the motor
        self.msg_r = String()
        self.msg_l = String()
        
        super().__init__('drive_node')
        #Self clock
        self.timer = self.get_clock()
        self.publish_timer = [0,0] #First is new publish time, second is previous publish time

        self.publisher = self.create_publisher(
            String,
            'mcu_outgoing',
            10)
        self.publishing = False
        self.published = [[0,0,0,0],[0,0,0,0]]
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.convert_twist_to_tickfreq,
            10)
        self.create_timer(0.01, self.publish_ticks)
        self.subscription  # prevent unused variable warning
        self.publisher  # prevent unused variable warning

    def convert_twist_to_tickfreq(self, msg):
        '''
        This function converts the twist message to right and left wheel tick frequency
        '''
        if self.publishing:
            return
        lin_v = msg.linear.x
        self.ang_w = msg.angular.z
        #print(f'lin_v: {lin_v}, ang_w: {self.ang_w}')

        #Calculate right and left wheel linear velocity
        self.v_l = (lin_v - self.ang_w*self.b/2)
        self.v_r = (lin_v + self.ang_w*self.b/2)
        #Update r_icc
        try:
            self.r_icc = self.b * (self.v_l + self.v_r)/(2*(self.v_r - self.v_l))
        except Exception as e:
            self.r_icc = None

        #Calculate right and left wheel angular velocity
        w_l = (lin_v - self.ang_w*self.b/2)/self.r
        w_r = (lin_v + self.ang_w*self.b/2)/self.r
        #Calculate right and left wheel tick frequency
        f_l = w_l/(2*math.pi)*self.ticks_per_rev
        f_r = w_r/(2*math.pi)*self.ticks_per_rev
        #Reduce the frequency to 3 decimal places
        f_l = round(f_l,3)
        f_r = round(f_r,3)
        #Create string message
        self.msg_r.data = f's r {math.copysign(1,f_r)} {abs(f_r)}'
        self.msg_l.data = f's l {math.copysign(1,f_l)} {abs(f_l)}'
    
    def publish_ticks(self):
        self.publishing = True
        if self.msg_r.data == '' or self.msg_l.data == '':
            self.publishing = False
            return
        self.publisher.publish(self.msg_r)
        self.publisher.publish(self.msg_l)
        self.published[0] = [self.v_r, self.v_l, self.ang_w, self.r_icc]
        self.publish_timer[1] = self.publish_timer[0]
        self.publish_timer[0] = self.timer.now().nanoseconds/1e9
        
        if self.publish_timer[1] == 0:
            self.publish_timer[1] = self.publish_timer[0]
            self.published[1] = self.published[0]
            
        else:
            dt = self.publish_timer[0] - self.publish_timer[1]
            if self.published[1][3] == None:
                self.x = self.x + self.published[1][0]*dt*math.cos(self.theta)
                self.y = self.y + self.published[1][1]*dt*math.sin(self.theta)
            else:
                self.theta += dt*self.published[1][2]
                self.x = self.x + self.published[1][3]*(math.sin(self.theta) - math.sin(self.theta - dt*self.published[1][2]))
                self.y = self.y - self.published[1][3]*(math.cos(self.theta) - math.cos(self.theta - dt*self.published[1][2]))
            print(f'v_r: {self.v_r}, v_l: {self.v_l}, ang_w: {self.ang_w}, r_icc: {self.r_icc}, dt: {dt}')
            print(f'x: {self.x}, y: {self.y}, theta: {(self.theta)%(math.pi*2)*180/math.pi}')
            self.published[1] = self.published[0]

            self.msg_r.data = ''
            self.msg_l.data = ''
            self.publishing = False

    


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
import rclpy
import math 
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, Pose
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
        super().__init__('drive_node')

        #Robot position
        self.x = 0.0
        self.y = 0.0
        self.v_r = 0.0
        self.v_l = 0.0
        self.ang_w = 0.0
        self.theta = 0.0
        self.dt = 0.0
        self.publish_timer = [0.0,0.0] #First is new publish time, second is previous publish time

        #Robot parameters
        self.b = 0.295 #Distance between the wheels
        self.r = 0.085/2 #Radius of the wheel
        self.ticks_per_rev = 600 #Number of ticks per revolution of the motor
        self.msg_r = String()
        self.msg_l = String()
        #Self clock
        self.timer = self.get_clock()

        self.publisher = self.create_publisher(
            String,
            'mcu_outgoing',
            10)
        self.publishing = False
        self.published = [0.0,0.0,0.0,0.0]

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.convert_twist_to_tickfreq,
            10)
        self.create_timer(0.1, self.publish_ticks)
        self.subscription  # prevent unused variable warning
        self.publisher  # prevent unused variable warning

        self.pose_publisher = self.create_publisher(
            Pose,
            'pose',
            10)
        self.pose_publisher  # prevent unused variable warning

    def convert_twist_to_tickfreq(self, msg):
        '''
        This function converts the twist message to right and left wheel tick frequency
        '''
        if self.publishing:
            return
        lin_v = msg.linear.x
        self.ang_w = msg.angular.z

        #Calculate right and left wheel linear velocity
        self.v_l = (lin_v - self.ang_w*self.b/2)
        self.v_r = (lin_v + self.ang_w*self.b/2)

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
        if self.published[3] == 0:
            self.published = [self.v_r, self.v_l, self.ang_w, self.timer.now().nanoseconds/1e9]
            self.publishing = False
            self.msg_r.data = ''
            self.msg_l.data = ''
            return
        self.integrate_pose(self.published[0], self.published[1], self.published[2], self.timer.now().nanoseconds/1e9 - self.published[3])
        self.published = [self.v_r, self.v_l, self.ang_w, self.timer.now().nanoseconds/1e9] #Store the new values for next integration
        self.publish_pose()
        self.msg_r.data = ''
        self.msg_l.data = ''
        self.publishing = False

    def integrate_pose(self, v_r, v_l, w, dt):
        '''
        This function integrates the pose of the robot using the right and left wheel velocity
        '''
        if v_r == v_l:
            self.x += v_r * dt * math.cos(self.theta)
            self.y += v_r * dt * math.sin(self.theta)
            self.theta += w * dt
        else:
            R = self.b/2 * (v_r + v_l)/(v_r - v_l)
            ICC = [self.x - R*math.sin(self.theta), self.y + R*math.cos(self.theta)]
            x = self.x - ICC[0]
            y = self.y - ICC[1]
            x_n = x * math.cos(w*dt) - y * math.sin(w*dt)
            y_n = x * math.sin(w*dt) + y * math.cos(w*dt)
            self.x = x_n + ICC[0]
            self.y = y_n + ICC[1]
            self.theta += w * dt
    
    def publish_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        q = euler_to_quaternion(0.0, 0.0, self.theta)
        pose.orientation = q
        self.pose_publisher.publish(pose)

def euler_to_quaternion(roll, pitch, yaw):
    from math import sin, cos
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

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
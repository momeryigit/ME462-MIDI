import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        #Subscribe to /pose topic and call pose_callback when a message is received
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, qos_profile)

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # Joint state publisher for further implementation
        # joint_state = JointState()
        # self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

    def pose_callback(self, msg):
        self.get_logger().info(f'Pose: x: {msg.position.x}, y: {msg.position.y}, theta: {quaternion_to_yaw(msg.orientation)}')
        #Create message type and fill it with data
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'world_frame'
        odom_trans.child_frame_id = 'base_link'
        position = msg.position
        orientation = msg.orientation
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.transform.translation.x = position.x
        odom_trans.transform.translation.y = position.y
        odom_trans.transform.translation.z = position.z
        odom_trans.transform.rotation = orientation

        # Send the transform
        self.broadcaster.sendTransform(odom_trans)

        #         # update joint_state
        #         now = self.get_clock().now()
        #         # joint_state.header.stamp = now.to_msg()
        #         # joint_state.name = ['swivel', 'tilt', 'periscope']
        #         # joint_state.position = [swivel, tilt, height]

        #         # update transform
        #         # (moving in a circle with radius=2)
        #         odom_trans.header.stamp = now.to_msg()
        #         odom_trans.transform.translation.x = cos(angle)*2
        #         odom_trans.transform.translation.y = sin(angle)*2
        #         odom_trans.transform.translation.z = 0.7
        #         odom_trans.transform.rotation = \
        #             euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

        #         # send the joint state and transform
        #         #self.joint_pub.publish(joint_state)
        #         self.broadcaster.sendTransform(odom_trans)

        #         # Create new robot state
        #         tilt += tinc
        #         if tilt < -0.5 or tilt > 0.0:
        #             tinc *= -1
        #         height += hinc
        #         if height > 0.2 or height < 0.0:
        #             hinc *= -1
        #         swivel += degree
        #         angle += degree/4

        #         # This will adjust as needed per iteration
        #         loop_rate.sleep()

        # except KeyboardInterrupt:
        #     pass

def quaternion_to_yaw(q):

    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
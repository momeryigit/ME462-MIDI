# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from .submodules.serialcom import burdayim
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class MinimalSubscriber(Node):
    '''
    Creates a subscriber to subscribe to cmd_vel topic. 
    '''

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscribed to cmd_vel topic')


    def listener_callback(self, msg):
        linear_velocity = msg.linear
        angular_velocity = msg.angular
        burdayim()
        self.get_logger().info('Linear Velocity: [%f, %f, %f]' % (linear_velocity.x, linear_velocity.y, linear_velocity.z))
        self.get_logger().info('Angular Velocity: [%f, %f, %f]' % (angular_velocity.x, angular_velocity.y, angular_velocity.z))


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

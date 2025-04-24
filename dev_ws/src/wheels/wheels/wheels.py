import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import random


class WheelsPublisher(Node):

    def __init__(self):
        super().__init__('WheelsPublisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            String, 
            "moveWheels",
            self.listener_callback,
            10
        )
        
        # I believe you have to keep sending Twist messages to keep the wheels moving instead of a single message.
        #    -> I could be wrong :), in that case remove timer and publish message at end of listener_callback function.
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.speed = 0.2 # In m/s 
        self.moving = False
        self.twist = Twist()
        
    def listener_callback(self, msg):
        twist = Twist()

        if msg.data == "moveForward":
            twist.linear.x = self.speed
            twist.angular.z = 0.0
            self.moving = True
        elif msg.data == "moveBackward":
            twist.linear.x = -self.speed
            twist.angular.z = 0.0
            self.moving = True
        elif msg.data == "rotateCCW":
            twist.linear.x = 0.0
            twist.angular.z = self.speed
            self.moving = True
        elif msg.data == "rotateCW":
            twist.linear.x = 0.0
            twist.angular.z = -self.speed
            self.moving = True
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.moving = False
    
    def timer_callback(self):
        if self.moving:
            self.publisher_.publish(self.twist)
            self.get_logger().info('Published Twist message: %s', self.twist)
        else:
            self.twist = Twist()  # Default stationary twist message (i think :))
            self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)

    publisher = WheelsPublisher()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

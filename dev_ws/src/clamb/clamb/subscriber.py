import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

#hightech hs322HD type servo
#TODO WHICH PIN TO USE
SERVO_PIN = 18 #The GPIO Pin connected to the clamp

class ClampSubscriber(Node):
    def __init__(self):
        super().__init__('clamp_subscriber')
        self.subscription = self.create_subscription(
        String,
        'topic',
        self.listener_callback,
        10)
        self.subscription # prevent unused variable warning
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN,50)
        self.pwm.start(0)
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 'open':
            self.open_clamp()
        elif msg.data == 'close':
            self.close_clamp()
        else:
            self.get_logger().info('This data is not compatible: "%s"' % msg.data)
    def open_clamp(self):
        self.get_logger().info('opening...')
        self.set_servo_angle(90)
        #GPIO spin to command servo 
    def close_clamp(self):
        self.get_logger().info('closing...')
        self.set_servo_angle(0)
    def set_servo_angle(self,angle):
        duty_cycle = 2 + (angle / 18)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(2) # Wait for servo to finish (this time is randomly chosen right now)
        self.pwm.ChangeDutyCycle(0) #Stop servo
    def destroy(self):
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    clamp_subscriber = ClampSubscriber()
    try:
        rclpy.spin(clamp_subscriber)
    except KeyboardInterrupt:
        clamp_subscriber.destroy()
    finally:
        clamp_subscriber.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import sys
import time
from std_msgs.msg import Int32MultiArray

#uncommand voor pins
#import RPi.GPIO as GPIO

# assign GPIO Pins
#GPIO_TRIGGER = 18
#GPIO_ECHO = 24


class UltrasonicPublisher(Node):

    def __init__(self): 
        #uncommand voor pins
        #self.trigger = GPIO_TRIGGER
        #self.echo = GPIO_ECHO
        
        self.timeout = 0.05
        
        #uncommand voor pins
        # GPIO Modus (BOARD / BCM)
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setwarnings(False)

        # Set direction of GPIO pins (IN --> Input / OUT --> Output)
        #GPIO.setup(self.trigger, GPIO.OUT)
        #GPIO.setup(self.echo, GPIO.IN)

        super().__init__('ultrasound_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'read_ultrasound_sensor', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.read_sensor)

    def read_sensor(self):
        msg = Int32MultiArray()

        data = []
        data.append(self.distance())
        data.append(self.speed())

        msg_data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Distance = {data[0]} cm and \n speed = {data[1]} m/s')

    def distance(self):
#-------uncommand voor pins-------------------------------------------
        #GPIO.output(self.trigger, True) # set trigger to HIGH

        # set trigger after 0.01 ms to LOW
        #time.sleep(0.00001)
        #GPIO.output(self.trigger, False)

        #startTime = time.time()
        #arrivalTime = time.time()

        #timeout_start = time.time()

        # store startTime
        #while GPIO.input(self.echo) == 0:
            #startTime = time.time()

            #if startTime - timeout_start > self.timeout:
                #return -1

        # store arrivalTime
        #while GPIO.input(self.echo) == 1:
            #arrivalTime = time.time()

            #if startTime - timeout_start > self.timeout:
                #return -1
#----------------------------------------------------------------------

#-------test-----------------------------------------------------------
        timeout_start = time.time()
        time.sleep(0.00052)
        startTime = time.time()

        if startTime - timeout_start > self.timeout:
               return -1

        # store arrivalTime
        time.sleep(0.0005)
        arrivalTime = time.time()

        if startTime - timeout_start > self.timeout:
                return -1
#---------------------------------------------------------------------        
        
        if startTime != 0 and arrivalTime != 0:
            # calculate the difference between start and stop
            duration = arrivalTime - startTime

            # multiply with speed of sound (34300 cm/s)
            # and divide by 2 because there and back
            distance = (duration * 34300) / 2 #cm

            if distance >= 0:
                return distance #cm 
            else:
                return -1
        else:
            return -1

    def speed(self):
        start_time = time.time()

        start_distance = self.distance() 
        end_distance = self.distance() 

        end_time = time.time()

        speed = (end_distance - start_distance) / 1.0   # m/s
        #speed = (end_distance - start_distance) / (start_time-end_time)   # m/s

        return speed

def main(args=None):
    rclpy.init(args=args)
    ultrasound_publisher = UltrasonicPublisher()
    rclpy.spin(ultrasound_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

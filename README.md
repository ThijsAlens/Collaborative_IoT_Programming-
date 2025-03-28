# Collaborative IoT Programming-

## Running the package

To run the package use the following command:

```ros2 run wheels WheelsPublisher``` 

## Who, what, how?

### Sending information to move the wheels

To move the wheels send one of the following Strings to the "moveWheels" topic:

- moveForward

- moveBackward

- rotateCW

- rotateCCW

- stop

This info can also be found in [interface.txt](https://github.com/ThijsAlens/Collaborative_IoT_Programming-/blob/wheels/interface/interface.txt).

### General information

The wheels package uses a timer to repeatedly send a Twist message to continously move the wheels.
This may or may not be necessary, if not, remove the timer and timer_callback and place the publish method at the end of the listeren_callback function.
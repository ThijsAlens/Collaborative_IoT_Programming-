# Collaborative IoT Programming-

## Available topics
- **getImage**, *type: sensor_msgs.msg.Image*, consists of the frames written by the camera.  
- **arucoDetection**, *type: custom_msg_aruco.msg.PositionStatus*, consist of a bool (found) and a position (x and y)
- **moveWheels**, *type: String*, Send: moveForward | moveBackward | rotateCCW | rotateCW | stop. 

## Running the packages

### Run the camera
```ros2 run camera_read ImagePublisher``` 

Camera frames are uploaded to the topic **getImage**

### Run ball_inference 
```ros2 run ball_inference ball_inference```

### Run Aruco_detector
```ros2 run aruco_detection detector```

### Run Wheels package
```ros2 run wheels WheelsPublisher```

To move the wheels send one of the following Strings to the "moveWheels" topic:
 
 - moveForward
 
 - moveBackward
 
 - rotateCW
 
 - rotateCCW
 
 - stop

### Run ultrasound
```ros2 run ultrasound USPublisher```

It publishes an array of speed and distance. Speed first then distance.

## Custom msgs found in ```custom_msg_aruco.msg```

**PositionStatus** consists of:
- *found [Bool]*  
- *x [Int32]*  
- *y [Int32]* 

**UltrasoundData** consists of:
- *speed [Int32]*  
- *distance [Int32]* 


# Collaborative IoT Programming-

## Available topics
- **getImage**, *type: sensor_msgs.msg.Image*, consists of the frames written by the camera.  
- **arucoDetection**, *type: custom_msg_aruco.msg.PositionStatus*, consist of a bool (found) and a position (x and y)

## Running the packages

### Run the camera
```ros2 run camera_read ImagePublisher``` 

Camera frames are uploaded to the topic **getImage**

### Run ball_inference 
```ros2 run ball_inference ball_inference```

## Custom msgs found in ```custom_msg_aruco.msg```

**PositionStatus** consists of:
- *found [Bool]*  
- *x [Int32]*  
- *y [Int32]*   

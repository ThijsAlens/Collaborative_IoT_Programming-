# Collaborative IoT Programming-

## Available topics
> **getImage**, *type: sensor_msgs.msg.Image*, consists of the frames written by the camera. 

## Running the packages

### Run the camera
```ros2 run camera_read ImagePublisher``` 

Camera frames are uploaded to the topic **getImage**

### Run ball_inference 
```ros2 run ball_inference ball_inference```

### Run Aruco_detector
```ros2 run aruco_detection detector```

import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        
        
        launch_ros.actions.Node(
            package = 'camera_read',
            executable = 'ImagePublisher',
            name = 'camera_read'
        ),
        launch_ros.actions.Node(
            package = 'ball_inference',
            executable = 'ball_inference',
            name = 'ball_inference'
        ),
        launch_ros.actions.Node(
            package = 'aruco_detection',
            executable = 'detector',
            name = 'aruco_decetion'
        ), 
        launch_ros.actions.Node(
            package = 'clamp',
            executable = 'clampSubscriber',
            name = 'clamp'
        ), 
        launch_ros.actions.Node(
            package = 'wheels',
            executable = 'WheelsPublisher',
            name = 'wheels'
        ),
        launch_ros.actions.Node(
            package = 'ultrasound',
            executable = 'USPublisher',
            name = 'ultrasound'
        ),
        launch_ros.actions.Node(
            package = 'Controller',
            executable = 'Controller',
            name = 'Controller'
        )
    ])



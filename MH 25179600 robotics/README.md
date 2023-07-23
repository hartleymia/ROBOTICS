# ROBOTICS ASSIGNMENT 

## Mia Hartley 25179600 <<25179600@students.lincoln.ac.uk>>

### 1.    A (max 100 word) summary of your solution (describing what the system is doing). 

In the OpencvBridge script I have used colour thresholding to detect the colourful objects in the simulation. Colour thresholding involves selecting a range of colours from the image and setting a binary threshold that marks pixels within that range as white and the rest as black. 

I have also used the laser scan of the turtlebot to control its movement. It moves forwards until an obstacle is within a certain distance, which causes the turtlebot to reverse and rotate to a different direction until the obstacle is no longer in the turtlebots way.


### 2.    An (optional) installation instruction, in case you use software components or ROS2 packages that are not installed in the provided Docker image.

to install opencv

enter this in the terminal
ros2 pkg create --build-type ament_python cv_basics --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2

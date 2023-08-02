# ROBOTICS ASSIGNMENT 

## Mia Hartley 25179600 <<25179600@students.lincoln.ac.uk>>

### 1.    A (max 100 word) summary of your solution (describing what the system is doing). 

#### Movement:
* Forwards
  > Default movement.
* Veering left/right
  > If object detected close to the front right or left of robot.
* Rotating left/right
  > robot stops and rotates left/right depending on where largest space is.
* Reversing
  > If wheels not moving, reverse.

#### Detection:
* Tilt detection
  > Odometry used to detect pitching/rolling.
* Oscillation detection.
  > detects left/right oscillation.
* Collision detection.
  > laser scanner used to determine direction to move by detecting objects around it.
* Coloured object detection.
  > Colour thresholding used to detect coloured objects

:::info
When all coloured objects detected, the application stops.
:::

### 2.    An (optional) installation instruction, in case you use software components or ROS2 packages that are not installed in the provided Docker image.

there are no additional components or packages installed

### 3.    An explanation on how to start your system, detailing all commands and steps necessary to reproduce and run your solution.

* set up/login to docker and open gazebo
* launch your chosen simulation environment 
* open robot.py in gazebos visual studio and run the script
* the solution should now be running

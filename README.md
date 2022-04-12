# warmup_project

## Driving in a Square

![drive_square.gif](https://github.com/Sshatzkin/warmup_project/blob/main/drive_square.gif)
### Description

I drove the robot in a square using only twist commands and ros.sleep commands to control the timing of each action.

A for loop iterate 4 times, and each time the robot drives forward for a short duration, turns left for a short duration, and then repeats.

### Code Explanation

I defined a DriveSquare class with 2 methods:

- __init__ - Initializes the ros node that will be used by the rest of the code
- __run__ - Creates 3 Twists, forward, left turn, and stop. Then, start a loop that executes the forward movement, followed by the left turn 4 times each. After the loop, execute a stop.

## Follow a Person

![follow.gif](https://github.com/Sshatzkin/warmup_project/blob/main/follow.gif)

### Description

The robot identifies the angle and distance to the nearest point identified by the LIDAR scanner, and it steers toward this point. The movement and rotation speed are modified via a proportional control mechanism relative to the angle and distance from target.

### Code Explanation

I defined a Follow class with 3 methods:

- __init__ - Initializes the ros node that will be used by the rest of the code
- __run__ - Initiates an endless loop with rospy.spin()
- __process_scan__ - This function is called every time the LIDAR scanner returns new data. It executes a few basic steps:
  - First, it checks if it is currently facing something within stopping distance directly in front of it. If so, destination has been reached. If not, then it must identify and move toward a target.
  - Using a call to get_min_nonZero, I retrieve the angle and distance to the nearest "thing" the LIDAR can see. I use this information to calculate a 0-1 distance error and a -1-1 angle error.
  - These error values are plugged into proportional control formulas for setting angular move speed and linear move speed, steering the robot faster toward an object which is further away.

I also defined the helper function __get_min_nonzero__, which takes an array of range values from the scanner and returns the angle and value of the smallest non-zero value (because 0 implies unmeasurable distance)

## Follow a Wall
![follow_wall.gif](https://github.com/Sshatzkin/warmup_project/blob/main/follow_wall.gif)

### Description

The robot follows along a wall on its left side. I achieve this through simple logic that has the robot maintain its position in a target distance range from the wall, and executing a right turn whenever it encounters an obstacle to the front.
### Code Explanation

I defined a Wall_Follower class with 3 methods:

- __init__ - Initializes the ros node that will be used by the rest of the code
- __run__ - Initiates an endless loop with rospy.spin()
- __process_scan__ - This function is called every time the LIDAR scanner returns new data. My process scan function can be broken up into a few distinct cases:
  - If no wall is directly in front of the robot...
    - If the robot is too far from the left wall, begin turning to the left
      - If nothing is close at 45 degrees to the left, this is likely a corner, so slow down forward movement and turn left quickly
    - If the robot is too close to the left wall, begin turning to the right
    - The robot is in the target range from the left wall, go straight faster
  - There is a wall directly in front of the robot, this must be a corner, so turn right
  - If something is close to the robot on right, stop. This is for testing purposes so the robot can be put in "timeout" on the desk between trials.

Additionally, I defined a helper function __alligned__, that is used to determine if the robot is in target range of the left wall. This funciton checks the distance from the wall at a few different angles on the left side and returns an allignment score from -1 to 1 to describe how close the robot is to the target range.

## Conclusions

### Challenges

The initially, my primary challenge was setting up my programming environment. I believe that I spent somewhere around 8 hours getting all of VMs working and ROS installed in a way that would successfully run and connect to the robot. My final configuration involved installing a clean Ubuntu VM using Virtual Box, and installing ROS and all of its dependencies on that.

The primary coding challenge for this assignment was tuning the parameters used to drive the robot. For each task, I was able to come up with a successful logic structure quite quickly, but these solutions were slow or resulted in the robot overshooting its targets. It took many iterations of tuning speeds and distances, as well as parameters used in my proportional control formulas.

### Future Work

If I had more time, I would have taken the time to build PID controllers for the person and wall following behaviors. Especially in the case of the person following behavior, using the rate of change of distance and angle to the person would allow the robot to follow much more efficient paths when following the person. The wall following robot could also benefit from this - a PID controller would allow the robot to more consistently allign itself with the target distance.

### Takeaways

1. Tuning the parameters of a control scheme is extremely important. Small differences in constants used to paramterize the control functions can make a very large difference in the resultant robot behavior.
2. Sensors are flawed. The LIDAR sensor on the turtlebot may sometimes return innacurate values and the scanner returns results at an unpredictable rate. This variability means that the robot often needs to move cautiously so as to not overshoot a target in between scans or after one innacurate reading leads it astray. In order for the robot to move faster without it being reckless, we would need to increase the scanning rate or include duplicate sensors that can compare results with each other.
3. Complex behaviors can be achieved with simple logic. The solutions I've submitted to each of these problems involve very basic logic for the robot to follow. The robot uses a few data points collected from the LIDAR sensor to take one of a small set of actions. The real complexity comes from the proportional control systems that adjust the severity of the robot's actions to its current environment state.

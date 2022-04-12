# warmup_project

## Driving in a Square

### Description

I drove the robot in a square using only twist commands and ros.sleep commands to control the timing of each action.

A for loop iterate 4 times, and each time the robot drives forward for a short duration, turns left for a short duration, and then repeats.

### Code Explanation

I defined a DriveSquare class with 2 methods:

- __init__ - Initializes the ros node that will be used by the rest of the code
- __run__ - Creates 3 Twists, forward, left turn, and stop. Then, start a loop that executes the forward movement, followed by the left turn 4 times each. After the loop, execute a stop.

### Gif

![drive_square.gif](https://github.com/Sshatzkin/warmup_project/blob/main/drive_square.gif)

## Follow a Person

### Description

The robot identifies the angle and distance to the nearest point identified by the LIDAR scanner, and it steers toward this point. The movement and rotation speed are modified via a proportional control mechanism relative to the angle and distance from target.

### Code Explanation
![follow.gif](https://github.com/Sshatzkin/warmup_project/blob/main/follow.gif)

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

The greatest challenge for this first project was setting up my programming environment. I believe that I spent somewhere around 8 hours getting all of VMs working and ROS installed in a way that would successfully run and connect to the robot. My final configuration involved installing a clean Ubuntu VM using Virtual Box, and installing ROS and all of its dependencies on that.

Driving the robot in a square was not a significant challenge, but it did require some tuning to find rotation speeds and durations that best approximate right angles. I played with a few different configurations before settling on the one included in my submission.

### Future Work

If I had more time, I would have achieved this project using the odometry approach because it would be much more helpful for achieving precise turns. With the timing approach I used, there is always a little bit of error because the robot's turns can be affected by friction with the surface it's driving on and other factors which lead to variability in each turn.

Using odometry data and external sensors could help the robot position itself better in the world, and make more precise movements tracing out a square.

It would also be interesting to build in features that allow the operator to specify a square size, and maybe a square position in the space, and have the robot trace out that shape.

### Takeaways

- Takeaway 1

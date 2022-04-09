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

The robot follows me __TO DO__
### Code Explanation

I defined a Follow class with 2 methods:

- __init__ - Initializes the ros node that will be used by the rest of the code
- __run__ - TODO

### Gif

![follow.gif](https://github.com/Sshatzkin/warmup_project/blob/main/follow.gif)


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

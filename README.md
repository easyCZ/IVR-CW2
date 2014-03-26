Introduction to Vision and Robotics 2014
======

*Mark Nemec, Milan Pavlik*

*s1140740, s1115104*


# 1 Introduction

## 1.1 The Task

The idea of the task is to design an algorithm to allow a Kephera robot to move through an environment while maintaining relatively constant distance from the walls and obstacles. The environment consists of a room with obstacles (boxes). The robot is required to carefully drive around the obstacles without hitting them. The robot continues following the walls until another obstacle is found.

The task is divided into two sections, in the first the robot is not required to stop at any point and keeps on driving around the room indefinitely. This is similar to a vacuum cleaner robot used in households. The second task is comparable to a surveillance robot where the robot traces around the walls, maintaining constant distance and avoiding obstacles, but it is required to stop when it completes a lap. It should be stopped on the same spot where it originally started.

The Kephera robot has two motors on each side of the robot as well as eight sensor positioned around the robot. The sensors can be used to retrieve the distances of objects around the robot.


## 1.2 Main Ideas

The main ideas used in the implementation are the following: *following a wall*, *avoiding obstacles* and *returning to the home location*.

Following a wall is based on the concept of a *Proportional Integral Derivative* (PID) controller where ideal situations are established and deviations from the ideal position are corrected for. Additionally, to the proportional error function, we also use the integral error (or cumulative error function) to influence the amount by which we correct for the ideal position error to speed the process. In essence, we do not use all the three parts of PID as the derivative component was not necessary.

In order to avoid obstacles in the path, we switch a *rotate* mode where the wheels of the robot are set to rotate on the spot. The *rotate* mode is exited when the sensor provide sufficient evidence of having rotated enough. In the next stage the PID controller is restarted and following a wall is resumed.

Returning to the home location is done through relative odometry calculations. Initially, the relative location of the robot is zero and every action updates the internal representation of position. As the robot approaches its home location, the relative location will be approaching zero and the robot stops.


#2 Methods

## 2.1 PI Controller

The PI Controller is based on proportional error from an optimal point and factors in the cumulative error of previous actions.

Firstly, we select a set point `set_point` which represents the ideal reading of a sensor. Secondly, we compute the proportional part of the PI controller, to do this we find the offset of the current reading `current_reading` from the ideal value. Finally, we use a value of gain, `p_gain` to fine tune the proportional component.

`proportional = (set_point - current_reading) * p_gain`

Secondly, we keep track of the cumulative error function, that is the past errors and their corrections. The error function allows us to influence the action that needs to be executed next. For example, if the error is high, the turn action executed next should be faster than if the error were low. The current error can be computed similarly to the *proportional* component. Initially, we find the offset and multiply by integral gain constant `i_gain`.

`integral = (set_point - current_reading) * i_gain`

Next, the cumulative error is updated with the value of the *integral* component.

`c_error = c_error + integral`

Finally, the return value of the PI Controller becomes the sum of the cumulative error and the proportional.

`pi = propotional + c_error`

The resulting action will be stronger if the error is high and vice versa. The actions will eventually stabilize in an equilibrium until an obstacle or non-straight line object is found.

## 2.2 Wall following




## 2.3 Obstacle Avoidance

## 2.4 Odometry

# 3 Results

## 3.1 Distance Control

## 3.2 Obstacle Avoidance

## 3.3 Returning home

# 4 Discussion

## 4.1 Future Improvements

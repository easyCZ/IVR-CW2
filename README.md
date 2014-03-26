Introduction to Vision and Robotics 2014
======

*Mark Nemec, Milan Pavlik*

*sXXXXXXX, s1115104*

# 1 Introduction

## 1.1 The Task

The idea of the task is to design an algorithm to allow a Keaphera robot to move through an environment while maintaining relatively constant distance from the walls and obstacles. The environment consists of a room with obstacles (boxes). The robot is required to carefully drive around the obstacles without hitting them. The robot continues following the walls until another obstacle is found.

The task is divided into two sections, in the first the robot is not required to stop at any point and keeps on driving around the room indefinitely. This is similar to a vacuum cleaner robot used in households. The second task is comparable to a surveillance robot where the robot traces around the walls, maintaining constant distance and avoiding obstacles, but it is required to stop when it completes a lap. It should be stopped on the same spot where it originally started.


## 1.2 Main Ideas

The main ideas used in the implementation are the following: *following a wall*, *avoiding obstacles* and *returning to the home location*.

Following a wall is based on the concept of a *Proportional Integral Derivative* (PID) controller where ideal situations are established and deviations from the ideal position are corrected for. Additionally, to the proportional error function, we also use the integral error (or cumulative error function) to influence the amount by which we correct for the ideal position error to speed the process. In essence, we do not use all the three parts of PID as the derivative component was not necessary.

In order to avoid obstacles in the path, we switch a *rotate* mode where the wheels of the robot are set to rotate on the spot. The *rotate* mode is exited when the sensor provide sufficient evidence of having rotated enough. In the next stage the PID controller is restarted and following a wall is resumed.

Returning to the home location is done through relative odometry calculations. Initially, the relative location of the robot is zero and every action updates the internal representation of position. As the robot approaches its home location, the relative location will be approaching zero and the robot stops.

#2 Methods

## 2.1 Wall following

## 2.2 Obstacle Avoidance

## 2.3 Odometry

# 3 Results

## 3.1 Distance Control

## 3.2 Obstacle Avoidance

## 3.3 Returning home

# 4 Discussion

## 4.1 Future Improvements

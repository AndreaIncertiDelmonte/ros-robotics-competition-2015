# Package ros-robotics-competition-2015
Ros node first place at [RIMLab](http://rimlab.ce.unipr.it/)'s robotics competition 2015.


## Installation

### with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/AndreaIncertiDelmonte/ros-robotics-competition-2015.git
```

## Quick start

#### Launch the node simulation mode

``` bash
roslaunch rimlab_robotics_competition_2015 competition_simulation.launch
```

## Hardware

#### Robot: [Pioneer 3DX](http://www.mobilerobots.com/researchrobots/pioneerp3dx.aspx)
#### Laser scanner: [Sick LMS100](http://www.sick.com/group/EN/home/products/product_news/laser_measurement_systems/Pages/lms100.aspx)

## Simulation

#### Simulator: [Stage](http://wiki.ros.org/stage)
#### Map: competition_map.pgm

## Competition details

**Competition field:** corridor adjacent to the robotics lab with obstacles in the center of 30/40 cm wide.

**Rules:** the robot must recognize and get to the nearest obstacle. After that, a random number will be extracted, if it will be even the robot must pass the obstacle on the right otherwise on the left.
After passing three obstacles, the robot has to proceed straight and the round will end when it will pass a line drawn on the floor.

## Implemented behaviors:

###### - Obstacle avoidance
###### - Stay in the middle
###### - Obstacle approach
###### - Obstacle exceed with wall following


## Licence
Apache License Version 2.0, January 2004 http://www.apache.org/licenses/

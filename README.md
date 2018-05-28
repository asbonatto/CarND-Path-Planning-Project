# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
---
### Contents

1. [Project Specifications](#project-specifications)
1. [Building the project](#build)
1. [Design](#design)

## Project Specifications
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

##  Building the project

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* [Spline.h](http://kluge.in-chemnitz.de/opensource/spline/)
  * A spline function contained in a single header file.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

Here is the data provided from the Simulator to the C++ Program

### Simulator data

The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
---

## Design

The code is based on three main classes : 
1. CSys : the Coordinate System class handles coordinate system transform, vector projection and lane mapping. It implements a Frenet coordinate frame by fitting a spline to the map waypoints. Normal and tangent vectors are smoothly defined from the splines derivatives.
1. JMT : Jerk Minimizing Trajectory class implements basic quintic polynomial fitting with option to fit position, velocity and acceleration or velocity, acceleration and jerk. It also provides a simple polynomial derivative function to for quick evaluation of v, a and j.
1. Planner : the Planner class is responsible for the high level trajectory planning. It handles ego localization and traffic data for prediction and tactical decisions such as lane switching and velocity specifications. This class uses functionality from CSys and JMT classes to provide a trajectory to the controller.

###  Frenet coordinates
[The Frenet or intrinsic coordinates](https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas) is a convenient way of representing the car movement. It essentially converts the movement into a straight-line representation,  making it easier to represent tactical calculations such as lane changes.
The component functions rx(s) and ry(s) are spline fitting the waypoints, with special treatment at the boundaries to avoid discontiuities in the interpolation. The tangent vector, defined by the derivatives of the components functions rx'(s) and ry'(s), implicitly defines the normal angle, making it unnecessary to use the normal vectors provided in the waypoints file.

###  Behavioral Planning
The tactical decisions are based on the straight-line view provided by the frenet coordinate system. Essentially the ego position and velocity is subtracted from traffic data in order to compute collision times. Then, lane costs are computed for adjacent lanes by adding the following terms :
1. Efficiency term to switch to faster lanes, calculated with the equation $ 1 - logit(lane_speeds/MAX_SPEED) $
2. Lateral movement penalty term to avoid changing lanes in case of equivalent alternatives, calculated with the equation  $0.005*pow(lane - current_lane, 2)$
3. Collision penalty term, computed with $fabs(l - current_lane)*pow(d, -2)$. 

Since the lane speed is computed by accomodate the max acceleration without collision, we can neglect the collision term for the current lane and use this term to explicitly avoid collisions.
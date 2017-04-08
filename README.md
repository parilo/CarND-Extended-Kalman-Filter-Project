# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

![Kalman filter demo](https://github.com/parilo/CarND-Extended-Kalman-Filter-Project/blob/master/kalman-filter-demo.png "Kalman filter demo")

This Kalman filter is used for fusion of measurements from lidar and radar of one particular object. Files:

* kalman_filter.cpp holds Kalman filter predict and update functions for LIDAR and Radar
* FusionEKF.cpp holds code that processes incoming LIDAR and Radar measurements
* tools.cpp contains code for convertion from cartesian to polar radar coordinates, calculation of jacobian for applying radar measurements. Also this file holds code for RMSE calculation over test data.
* data folder contains test measurements
* main.cpp reads measurements from specified testing data file and calculates RMSE (root mean squared error).

## Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

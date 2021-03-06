# Extended Kalman Filter 

Self-Driving Car Engineer Nanodegree Program

---

## Introduction
In this project an extended Kalman filter was utilized to estimate the state of a moving object in the udacity simulator with noisy lidar and radar measurements. 

The video below shows what the simulator looks like when a c++ script is using its Kalman filter to track the object. Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and Root Mean Squared Error (RMSE) values from its Kalman filter.

![udacity_exampleVideo](./udacity_exampleVideo.gif) 

## The Approach
1. Initializing Kalman filter variables
2. Predicting where the object of interest is going to be after a certain time step
3. Updating where the object currently is based on sensor measurements
   The predicting and updating steps repeat themselfs in a loop
4. Calculate the Root Mean Squared Error (RMSE) comparing the Kalman filter results with the provided ground truth

## Files and code
#### [/src/main.cpp:](https://github.com/JulePralle/SDC_Term2_Project1_ExtendedKalmanFilter/blob/master/src/main.cpp)
* communicates with the Udacity Simulator receiving data measurements
* calls a function to run the Kalman filter
* calls a function to calculate the RMSE
* reads in the data and sends a sensor measurement to FusionEKF.cpp

#### [/src/FusionEKF.cpp:](https://github.com/JulePralle/SDC_Term2_Project1_ExtendedKalmanFilter/blob/master/src/FusionEKF.cpp)
* initializes the filter
* calls the predict function
* calls the update function
* takes the sensor data and initializes and updates variables
* uses the efk_ instance to call the predict and update equations

#### [/src/kalman_filter.cpp:](https://github.com/JulePralle/SDC_Term2_Project1_ExtendedKalmanFilter/blob/master/src/kalman_filter.cpp)
* defines the predict function and both update functions for lidar and radar
* the KF class is defined in this file and in kalman_filter.h

#### [/src/tools.cpp:](https://github.com/JulePralle/SDC_Term2_Project1_ExtendedKalmanFilter/blob/master/src/tools.cpp)
* function to calculate the RMSE and the Jacobian Matrix





# Udacity Part
---
## Project Starter Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

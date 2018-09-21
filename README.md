# Car Path Planning Project
Self-Driving Car Engineer Nanodegree Program

---

## Overview

Goal of this project is make self driving car safely merge through traffic on a virtual highway. Car speed should be as close to 50 m/h as possible but not overdo it. All maneuvers have to be done nice and smooth.

## Compile

This project require [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) to be installed. This repository includes two files that can be used to set up and install it for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make

### Dependencies

* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4

## Run the Code

This project involves the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)


To run the Path Planner algorithm use

```

./build/path_planning
```

command in project directory.

## Reflection

We have two main part in Path Planner:

1. Analyze nearby traffic and choose the best line.
1. Create way points based on current and desired position.

### Analyze traffic situation

Code for this part is located in `AnalyzeTraffic` method of `PathPlaner` class.

We calculate pseudo-weight function for each line. This function has three components:

1. Small weight for outside lines. This makes car prefer to stay in the middle on the empty road line and have more capability for maneuver.

1. If there is a car in any line in front of our car with distance from 15 meters to 60 meters we prepare additional component for this line weight. This component is higher as this car is closer to our car. But it is added to the weight function only if there are any car closer then 40 m. This approach allow us to choose better line when the traffic situation is more or less clear.

1. We add high value to the weight function if there are car in line from 10 meters backward to 15 meters forward. This prevent our car to choose this line.

The line with lowest weight is returned as the desired line for path creation part. Also, `AnalyzeTraffic` method returns desired speed for chosen line based on the cars speeds in this line.

### Path creation

Code for this part is located in `Iterate` method of `PathPlaner` class.

First of all we create reference points based on previous car positions (or current position and heading angle) and few points in the future with respect to line chosen by `AnalyzeTraffic` method. Then we rotate coordinate frame to the car coordinate from for simplification of using spline and velocity estimation. After coordinate frame rotation we create spline going through way points in car coordinate system. Car velocity is calculated based on reference car velocity (which is also provided by `AnalyzeTraffic` method) and car maximum allowed acceleration/deceleration. Latter we add unused part of previous control points vector to the new vector and add new control points on generated spline trajectory with interval to keep selected car velocity.

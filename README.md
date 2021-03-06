# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

![pic](/samples/pic.png)
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Architecture

My solution can be seperated into 3 parts: surrounding checking, behavioral planning and path generation

### Surrounding Check

Included in `vehicle.cpp`,  function `Vehicle::surrounding_check`. 

This part is to check the status of the detected vechicles. The conditions are stored in 4 bool variables and 2 double variables: 

`Vehicle::car_left`: If there is a car at left side

`Vehicle::car_right`: If there is a car at right side

`Vehicle::car_ahead`: If there is a car ahead 

`Vehicle::car_ahead_close`: If the car ahead is close or not

`Vehicle::car_ahead_speed`: The speed of the ahead car

`Vehicle::car_ahead_s`: the location of the ahead car

### Behavioral Planning

Included in `vehicle.cpp`,  function `Vehicle::behavior_plan`. 

This part is to determine the action that the vehicle should take based on the surrounding checking results. 

|Condition                        |Action                  |
---------|--------
Car ahead close & no car at left  | Change to the left lane
Car ahead close & no car at right | Change to the left right
Car ahead not close but slow      | Slowly get close to the front car
Car ahead fast and at the center lane| Follow the front car
Not at the center lane and center lane available| Change to the center lane
No car ahead and slow| Speed up

The acceleration of the vehicle is carefully designed to fit different conditions and make the vehicle drive smoothly. 

### Path Generation

Included in `vehicle.cpp`,  function `Vehicle::path_gen`.

This part is to generate the path for driving based on the action determined above. I set 5 typicle points and use spline tools to generate the whole line. The whole process is listed as follows:

First check the size of the previous path, pick the last two points for continuty. 

Then generate another 3 points for path generation based on the action above: 
* The points are at the goal lane
* The points are obtained from 30m to 90m ahead at each 30m

Notice that we should convert the Frenet cooidinates to the Cartesian coordinates here. 

After making coordinates to local car coordinates, I use spline to fit the point set. Then I obtain about 50 points on the spline based on the velocity of the vehicle. 




## Rubic Points
* The code compiles correctly
* The car is able to drive more than 4.32 miles without any warnning. 
* The car stays in its lane, except for the time between changing lanes. And it can follow the front smoothly. 
* The car is able to change lanes smoothly. 

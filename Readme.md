# Introduction

This is a ROS1 Workspace that hosts the software that was written for a semester-long project, as part of the NUS module ME4232 (Small Aircraft and Unmanned Aerial Vehicles). It contains a single ROS Package: `me4232`

# Objective

The objective of the project was to design a simple simulator that can simulate position awareness and collision detection in a UAV swarm. Within the `me4232` package, there are two types of ROS nodes, found in the `src` director:

- UAV (uav.cpp) - simulates an unmanned aerial vehicle that estimates its own 3D odometry (position and velocity). Multiple of the same UAV nodes are generated to simulate a UAV swarm
- GCS (gcs.cpp) - simulates a ground control station that receives odometry data from each UAV, and uses these data to predict whether collision will take place

# Methodology

## UAV Position Estimation

Each UAV node is initialised in the simulator with a given 3D position (XYZ) and orientation (RPY). At each timestamp, it receives inertial navigation data: Linear acceleration and angular velocity, similar to an Inertial Measurement Unit (IMU) The node uses these sensor data to update its linear and angular position and velocity using integration techniques. The Euler rotation angle matrices are used to convert the sensor data from the UAV frame to the global frame, so that the 3D orientation (which is in global frame) can be accurately estimated.

(Note: An attempt to replace the Euler rotations with quaternions was tried but unsuccessful. The implementation can still be found in the UAV node file, but it is not implemented)

The UAV nodes package their position and velocity data into ROS Odometry and TF messages. These messages are published to the Odometry and TF2 topics to be obtained by the GCS.

## GCS Collision Detection

The GCS node receives odometry data from the Odometry and TF2 messages. It uses these data to generate a 3D ellipsoid for each UAV, which predicts the possible locations the UAV may be at a certain point in the future. If any of these ellipsoids is found to intersect, it means collision is likely to occur. THe GCS will then print a message on the terminal highlighting that collision will occur.

# Usage

The software should be run on ROS1 in an Ubuntu environment. It has been tested and verified to work on Ubuntu 18.04 + ROS Bionic, as well as Ubuntu 20.04 + ROS Noetic.

## Running the UAV nodes

Each UAV node obtains its IMU data from two possible sources:

- From a CSV file
- Randomly generate the data

### Preparing a CSV data file

To make the UAV node read from a csv file, uncomment the macro `#define CSV` at the top of the `uav.cpp` file.

The CSV data file's format is as follows

- The first row contains the initial 3D coordinates of the UAV, separated as x, y, z. For example, `0, 1, 0` initialises the UAV at x = 0m, y = 1m and z = 0m
- The subsequent rows contain the x, y, z linear acceleration (in m/s/s) and angular velocity (in rad/s) at that particular timestamp. For example, `1, 0, 0, 0, 0, 0.1` means that at that timestamp t, the UAV is acclerating in the x-direction at 1m/s/s, and rotating about the z-axis at 0,1rad/s.

The following CSV example initialises the UAV at x = 1, y = 2, z = 2, then acclerates the UAV forward in the x-direction for a certain amount of time, before continuing in a straight line at constant velocity.

```
1,2,2
1,0,0,0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
0,0,0,0,0,0
0,0,0,0,0,0
0,0,0,0,0,0
0,0,0,0,0,0
```

The following example initialises the UAV at x = 0, y = 0 and z = 0, acclerates the UAV forward in the x-direction for a certain amount of time, then continues at constant speed while pitching upwards (rotating about y axis) at 0.1rad/s

```
0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
1,0,0,0,0,0
0,0,0,0,0.1,0
0,0,0,0,0.1,0
0,0,0,0,0.1,0
0,0,0,0,0.1,0
```

### Generating random odometry data

To make the UAV node generate random odometry data, comment out the macro `#define CSV` at the top of the `uav.cpp` file.

The boundaries of the generated data are set using the follwoing macros, which can also be found at the top of the `uav.cpp` file: `MAX_INITIAL_POS` (Max initial position in any axis), `MAX_ACC` (Max linear acceleration in any axis), `MAX_ANG_VEL` (Max angnular velocity in any axis)

### Launching the UAV nodes

Use the `uav.launch` file located in the `launch` folder of the `me4232` package. The launch file will launch the following:

- UAV nodes. One node is launched for each UAV; add additional nodes with extra IDs where neccessary
- Rviz. This GUI helps to visualise the position of the UAVs in 3D space, making it more intuitive for human users
- me4232_rviz: This supplements a marker to each UAV in the Rviz environment to make the UAVs stand out more easily

The default configuration of the Rviz can be found in `rviz/config`; it is currently configured to display odometry data of each UAV node.

## Running the GCS nodes

Running and launching the GCS node is easier. Use the `gcs.launch` file located in the `launch` folder of the `me4232` package. The file should be launched in a separate terminal from the `uav.launch` file.

When the GCS is launched, it prints on the terminal whether any collision has taken place, at each timestamp.

# Acknowledgements

We will like to thank the ME4232 module coordinator, Dr. Sutthiphong Srigrarom, for his supervision and advice. The development team consists of Niu Xinyuan and Lau Yan Han from the National University of Singapore, Department of Mechanical Engineering.
# Kidnapped Vehicle (Particle Filter) project

This project implements a 2 dimensional particle filter in C++. Particle filter performs localization of the vehicle based on landmarks and initial information about vehicle position and orientation. The filter performs the following steps:

* Initialization

Initial localization information provided by the simulator and set to particles with noise.

* Prediction

Each particle is representing possible position of a car. On this step we make a prediction for each particle of the next car position and orientation based on the process model. 
Position of each particle will be highly likely different due to noise introduced by normal distribution as addition after the prediction.

* Update

Update step performed with measurement data. On this step filter gets information about the observations of landmarks from the sensors. Each landmark observation has noisy coordinates of the 
observed map landmarks. Filter performs coordinate transformation for each observation, this is required since measurement of the sensor is performed in the car's coordinate system while landmakrs are in the map coordinate system. Next a set of landmarks is selected from the map according to a particle's position and sensor range measurement.
Now filter have 2 vectors of observable landmarks and found observations from the sensors. Filter performs data association based on nearest neighbour algorithm. In other words filter
search in 2 vectors to find pairs of nearest map landmark and observed landmark.
Based on these data density probability is estimated for each pair using multivariate gaussian probability. Final weight of particle is defined as a multiplication of these density probabilities.

* Resample

On this step filter allows to "survive" particles according their weights. The higher the weight the higher likelihood for particle to "survive" in resampling process.
It is performed by a discrete distribution function of C++ standard library, which helps to randomly generate indexes of the array based on the array probability values.

[image1]: ./output_video.gif

Example of the filter performance on the simulator

![alt text][image1]

## Success Criteria

1. **Accuracy**:  particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.
The particle filter passes the accuracy with error values x: 0.170 y: 0.180 yaw: 0.006

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.
The particle filter passes the prformance by completing in 60 seconds



## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"] 

["sense_y"] 

["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"] 

["sense_observations_y"] 


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label 

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Src folder structure
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
See `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id



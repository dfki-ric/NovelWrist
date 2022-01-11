# NovelWrist.jl
A julia package for the kinematic analysis of the 2SPU + 2RSU + 1U mechanism that is used as wrist for the humaniod robot RH5, developed at the [DFKI Robotics Innovation Center](https://robotik.dfki-bremen.de/en/startpage.html). 

## Introduction
The 2SPU + 2RSU + 1U design and its kinematic analysis is presented in [link to paper](). This repository contains a more generalized version of the code that was used to perform the kinematic analyis. It offers the functionality to create designs of equal type by specifying geometric parameters and 
then computing (and visualizing), amongst others, 
![test](./docs/kinematic_analyis.png?raw=true "kinematic characteristics")

- the size of the feasible work space (a)
- the conditioning of the mechanism (b),
- and torque and speed values of the end-effector (c)

As additional feature these characteristics can be compared to the ones of a 2SPU + 1U mechanism, a conventional wrist design (also (c)).      
## Installation
```
pkg> add NovelWrist
```

## Documentation
### Create a new design 
```
using NovelWrist
build_wrist = WristGeometry(l = (0.045, 0.045), 
                            r = (0.049, 0.049), 
                            r_ = (0.049, 0.049),
                            h = (0.012, 0.012),
                            b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                            c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                            e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                            n = ([1, 0, 0], [-1, 0, 0]),
                            actuator_limits = ((0.113, 0.178), (0.113, 0.178)))
```
Here the example values that lead to the kinematic characteristics shown in the above plots have been assigned. Every geometric parameter is named in accordance to the kinematic model. 
![test](./docs/kinematic_model.png?raw=true "kinematic model")
The actuator limits denote the minimum and maximum values that can be reached by the linear actuators, denoted as 'q'.
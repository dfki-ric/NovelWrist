# NovelWrist.jl
A julia package for the kinematic analysis of the 2SPU + 2RSU + 1U mechanism that is used as wrist for the humaniod robot RH5, developed at the [DFKI Robotics Innovation Center](https://robotik.dfki-bremen.de/en/startpage.html). 

## Introduction
The 2SPU + 2RSU + 1U design and its kinematic analysis is presented in [link to paper](). This repository contains a more generalized version of the code that was used to perform the kinematic analyis. It offers the functionality to create designs of equal type by specifying geometric parameters and 
then computing (and visualizing), amongst others, 
![test](./docs/kinematic_analyis.png?raw=true "kinematic characteristics")

- the size of the feasible work space (a)
- the conditioning of the mechanism (b),
- and torque and speed values of the end-effector (c)

As additional feature these characteristics can be compared to the ones of a 2SPU + 1U mechanism, a conventional wrist design.      

## Documentation


# NovelWrist.jl
A julia package for the kinematic analysis of the 2SU\[RSPU\] + 1U mechanism that is used as wrist for the humaniod robot RH5v2, developed at the [DFKI Robotics Innovation Center](https://robotik.dfki-bremen.de/en/startpage.html). 

## Introduction
![test](./docs/2SPU+2RSU+1U_design_2.png?raw=true "CAD design of 2SU[RSPU] + 1U mechanism")

The 2SU\[RSPU\] + 1U design and its kinematic analysis is presented in [link to paper](). This repository contains a more generalized version of the code that was used to perform the kinematic analysis. It offers the functionality to create designs of equal type by specifying geometric parameters and 
then computing (and visualizing), the differential kinematics as well as the mechanisms conditioning in the work space. As additional feature these characteristics can be compared to the ones of a 2SPU + 1U mechanism, a conventional wrist design.      

## Installation
```jl
pkg> add NovelWrist
```

## Documentation
### Create a new design 
![test](./docs/kinematic_model.png?raw=true "kinematic model")

The geometric parameters that can be altered by the user in order to create a new design are named in accordance to the kinematic model. The below values (in mm) are those used througout the documentation process and are from the actual build wrist. 

```jl
julia> using NovelWrist

julia> build_wrist = WristGeometry(l = (0.045, 0.045), 
                                r = (0.049, 0.049), 
                                r_ = (0.049, 0.049),
                                h = (0.012, 0.012),
                                b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                                c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                                e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                                n = ([1, 0, 0], [-1, 0, 0]),
                                actuator_limits = ((0.113, 0.178), (0.113, 0.178))); 
```

The actuator limits denote the minimum and maximum values that can be reached by the linear actuators, denoted as 'q' in the kinematic model.

### Kinematics
#### Inverse and Forward Kinematics 
Compute one of 4 possible inverse kinematics solutions (2 for each side) in zero position of the end-effector:

```jl
julia> x = [0, 0]; # angles in rad  

julia> q = inverse_kinematics(x, build_wrist; specsol = [1,2], intrinsic = true) # constellations of 1 and 2 in specsol lead to 2^2 solutions
2-element Vector{Real}:
 0.13347357815533836
 0.13347357815533836
```

The obtained values for `q` correspont to the lower `actuator_limits`.  

#### Constrained Jacobian
To get the Jacobian **J** as product of the inverted work space Jacobian *J*x and the joint space Jacobian *J*q:

```jl
julia> J = Jacobian(x, build_wrist; specsol = [1,2], intrinsic = true, split = false)
2×2 Matrix{Real}:
 15.9633   15.9633
 17.737   -17.737
```
When `split = true`, **J**x and **J**q are returned componentwise. 

### Performance Analysis
The **condition index** of the mechanism plotted over `α` and `γ`, denoted as inclination and tilt angle in the paper, respectively.

```jl
julia> plot_conditioning(build_wrist, α = (-π, π), γ = (-π, π), specsol = [1,2], resol = 500) # increasing resol will give a higher resolution
```
![test](./docs/condition_index.png?raw=true "Conditioning")
The dashed lines indicate the workspace limits imposed by `actuator_limits`.

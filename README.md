# thesis-example
Αn online identification experiment of a 2-DOF robotic manipulator. 

## Summary
This work is a system identification study, and it's main purpose is the identification of a non-linear conitnuous system 
on a region of interest. The work is based on  Prescribed Performance Control (PPC) [1], Radial Basis Neural Networks 
(RBF-NNs) [2] and the results of [3] on the satisfaction of the Persistancy of Excitation condition for RBF-NNs.


## How to launch
To run this simulation, you will need MATLAB (at least R2015b). So, clone the folder and run the script as follows:
```matlab
cd demo
robot_example
% ... wait a lot ...
```

To plot the weights convergence you can edit and run the ```sig_recreation.m``` script inside ```demo/```.

## Files and their purpose
In this repository you will find:
- ```util/``` : Folder containg the utility functions like implementation of the PPC functions and RBF-NNs
- ```demo/```: Identification experiment files
    - ```demo_experiment/```: Results folder
    - ```robot_example.m```: Experiment definition and tunable parameters
    - ```sim_controller.m```: Invokes ODE and handels signal storage and reinitiallization in every iteration
    - ```robot_plant.m```: The simulated system equations
    - ``` robotic_2dof_idnt.m```: The control/identification loop
    - ```index_calcluator.m```: Active states calculator for accelerating the simulation
    - ```sig_recreation.m```: Plotting tool to evaluate the weights convergence

## References
[ [1] Robust adaptive control of feedback linearizable mimo nonlinear systems with prescribed performance](https://ieeexplore.ieee.org/document/4639441)\
[ [2] Universal approximation using radial-basis-function networks](https://ieeexplore.ieee.org/document/6797088)\
[ [3] Persistency of excitation in identification using radial basis function approximants](https://epubs.siam.org/doi/10.1137/S0363012992232555)

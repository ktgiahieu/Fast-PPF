# Fast-PPF
FastPPF is an implementation of the Fast and Robust Pose Estimation Algorithm for Bin Picking Using Point Pair Feature from [Mingyu Li et al.](https://ieeexplore.ieee.org/document/8545432)

The project takes a .STL mesh model as an input, then it will find every matches available in the scene provided as a .pcd file.

### Requirements
To build this project you will need:

* Point Cloud Library (PCL 1.7)

* Eigen library 3.0

* cmake 2.8

### Compile and run
```
    mkdir build
    cd build
    cmake ..
    make
    ./FastPPF
```
![Model](https://github.com/ktgiahieu/Fast-PPF/blob/master/images/model.PNG)

![Scene 0](https://github.com/ktgiahieu/Fast-PPF/blob/master/images/scene0.PNG)

![Scene 1](https://github.com/ktgiahieu/Fast-PPF/blob/master/images/scene1.PNG)

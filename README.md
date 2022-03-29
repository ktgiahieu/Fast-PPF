# Fast-PPF
FastPPF is an implementation of the [Fast and Robust Pose Estimation Algorithm for Bin Picking Using Point Pair Feature](https://ieeexplore.ieee.org/document/8545432) from [Mingyu Li](https://ieeexplore.ieee.org/author/37086527848) et al.

The project takes a .STL mesh model as an input, then it will find every matches available in the scene provided as a .pcd file.

### Experiment video
This is my Graduate Thesis video which uses this repo for object recognition.
[![Video(YouTube)](https://github.com/ktgiahieu/Fast-PPF/blob/master/images/video_cropped.png)](https://www.youtube.com/watch?v=udqj9vNyUDY)


### Requirements
CURRENTLY THERE IS A BUG WITH THE PCL IMPLEMENTATION OF PPF CALCULATION, PLEASE REFER TO [THIS ISSUE](https://github.com/ktgiahieu/Fast-PPF/issues/1#issuecomment-1080097065) TO REBUILD THE PCL LIBRARY TO OBTAIN CORRECT RESULTS.

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

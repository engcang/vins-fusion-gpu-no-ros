# VINS-Fusion-GPU version's No ROS version
<br>

#### Check other branch, ROS removing is done.
### VINS-Fusion : from [here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
### VINS-Fusion-GPU : from [here](https://github.com/pjrambo/VINS-Fusion-gpu)

<br>
<br>

#### No-ROS version
##### 0. camera_models : merged into one header, one source file
  + Original package is not dependent on ROS, using original one would be fine.
##### 1. Merged all header files into one.
##### 2. Merged all src files into one + main.
##### 3. Trying to remove ROS. ==> branch [here](https://github.com/engcang/vins-fusion-gpu-no-ros/tree/noros)
  + CameraPoseVisualization got removed
  + some Publishing functions got removed
  + using **ros_things.h**
  + need data set from EuRoc [here](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as ASL data format

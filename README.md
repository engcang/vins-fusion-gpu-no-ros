# VINS-Fusion-GPU version's No ROS version
<br>

### VINS-Fusion : [here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
### VINS-Fusion-GPU : [here](https://github.com/pjrambo/VINS-Fusion-gpu)

<br>
<br>
### No-ROS version
##### 0. camera_models : merged into one header, one source file
  + Original package is not dependent on ROS, using original one would be fine.
##### 1. Merged all header files into one.
##### 2. Merged all src files into one + main.
##### 3. ROS Removed [Done] 
  + CameraPoseVisualization got removed
  + some Publishing functions got removed
  + using **ros_things.h**
  + need data set from EuRoc [here](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as ASL data format

### BUILD
  + Install Ceres, Eigen [refer here](https://github.com/engcang/vins-application)
  + OpenCV version upper than 3.4.1 ( tested it on 3.4.1 and 3.4.3)
  ~~~shell
  $ git clone --single-branch -b noros https://github.com/engcang/vins-fusion-gpu-no-ros
  $ cd vins-fusion-gpu-no-ros/camera_models && mkdir build && cd build
  $ cmake .. && make
  $ cd ~/<cloned directory>/vins-fusion-gpu-no-ros/vins_estimator && mkdir build && cd build
  $ cmake .. && make
  ~~~

### EXECUTION
  ~~~shell
  $ cd ~/vins_gpu_ws && ./src/vins-fusion-no-ros-gpu/vins_estimator/build/vins_estimator ./src/vins-fusion-no-ros-gpu/config/mono.yaml ./data/image/left/data/ ./src/vins-fusion-no-ros-gpu/euroc_data_timestamp/MH01.txt ./data/imu/data.csv
  ~~~

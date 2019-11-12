# VINS-Fusion-GPU version's No ROS version
<br>

### VINS-Fusion : [here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
### VINS-Fusion-GPU : [here](https://github.com/pjrambo/VINS-Fusion-gpu)

<br>
<br>

### NO-ROS version

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
  + Have to edit config.yaml file -> refer original github
  + **show_TMI? : 1** makes node print more info.
  + **show_track :1** makes node using cv::imshow(tracked image with dotts and arrows)
  + **thread_affine :1** makes cpu affined with pthread function to allocate threads well
  ~~~shell
  $ cd ~/vins_gpu_ws && ./src/vins-fusion-no-ros-gpu/vins_estimator/build/vins_estimator ./src/vins-fusion-no-ros-gpu/config/mono.yaml ./data/image/left/data/ ./src/vins-fusion-no-ros-gpu/euroc_data_timestamp/MH01.txt ./data/imu/data.csv
  ~~~

<br>

### Update 19.08.13
  + supports **fish_eye** as supported in [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) , even not supported in VINS-Fusion ROS version 
  + supports **stereo** for dataset
  ~~~shell
  $ cd ~/vins_gpu_ws && ./src/vins-fusion-no-ros-gpu/vins_estimator/build/vins_estimator ./src/vins-fusion-no-ros-gpu/config/stereo.yaml ./data/image/left/data/ ./data/image/right/data ./src/vins-fusion-no-ros-gpu/euroc_data_timestamp/MH01.txt ./src/vins-fusion-no-ros-gpu/euroc_data_timestamp/MH01.txt ./data/imu/data.csv
  ~~~

### Update 19.09.18
  + DEBUG mode supports with **SHOW_TMI** parameter in config.yaml file. It prints out a lot more info.
  + FISHEYE mask reading method changed.

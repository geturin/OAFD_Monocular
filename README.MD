# OAFD_MONOCULAR
this work is my Master thesis research ：**3D mapping by a monocular camera with application to realtime obstacle avoidance control of quadrotor**

use monocular depth estimation and monocular visual SLAM to construct an autonomous flight system for a monocular-only quadrotor.
![](./data/my%20systeam%20en.drawio.svg)

https://user-images.githubusercontent.com/87827677/209362745-71a07fed-f7f8-49f1-a064-7b98f55e34b8.mp4

# [more experiment video](https://youtube.com/playlist?list=PLQYTjwXEt_RxlJJVSr9-HWyi-2qyCrkhP)

#  Prerequisites
Tested on Ubuntu 20.04 & 18.04

## [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
Use the ORB_SLAM3 as VSLAM(monocular)
- Install the [prerequisites](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites).
- Clone ORB-SLAM3:
```
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```
- Make changes to the source code if necessary to build successfully. For Ubuntu 20.04, you will need to change CMakeList from C++11 to C++14. I have incorporated the changes in [this fork](
https://github.com/thien94/ORB_SLAM3).
- Build:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
- Make sure that **`libORB_SLAM3.so`** is created in the *ORB_SLAM3/lib* folder. If not, check the issue list from the [original repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues) and retry.

### [orb_slam3_ros_wrapper](https://github.com/thien94/orb_slam3_ros_wrapper)

- Clone the package. Note that it should be a `catkin build` workspace.
```
cd ~/catkin_build/src/
git clone https://github.com/thien94/orb_slam3_ros_wrapper.git
```

- Open `CMakeLists.txt` and change the directory that leads to ORB-SLAM3 library at the beginning of the file (default is home folder `~/`)
```
cd ~/catkin_ws/src/orb_slam3_ros_wrapper/
nano CMakeLists.txt

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   $ENV{HOME}/ORB_SLAM3
)
```

- Build the package normally.
```
cd ~/catkin_build/
catkin build
```

- Next, copy the `ORBvoc.txt` file from `ORB-SLAM3/Vocabulary/` folder to the `config` folder in this package. Alternatively, you can change the `voc_file` param in the launch file to point to the right location.

- (Optional) Install `hector-trajectory-server` to visualize the trajectory.
```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

## [MiDAS](https://github.com/isl-org/MiDaS/tree/master/ros)
use the MiDAS as monocular depth estimation model
* create symlink for OpenCV:

```bash
sudo ln -s /usr/include/opencv4 /usr/include/opencv
```

* download and install MiDaS:

```bash
source ~/.bashrc
cd ~/
mkdir catkin_ws
cd catkin_ws
git clone https://github.com/isl-org/MiDaS
mkdir src
cp -r MiDaS/ros/* src

chmod +x src/additions/*.sh
chmod +x src/*.sh
chmod +x src/midas_cpp/scripts/*.py
cp src/additions/do_catkin_make.sh ./do_catkin_make.sh
./do_catkin_make.sh
./src/additions/downloads.sh
```
## [EGO_PLANNER](https://github.com/ZJU-FAST-Lab/ego-planner)
use the EGO_PLANNER as local planner

```
sudo apt-get install libarmadillo-dev
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```
# Installation
```
sudo apt-get install ros-$release-ros-numpy
sudo apt-get install ros-$release-tf2-sensor-msgs
cd catkin_ws/src
git clone https://github.com/geturin/OAFD_Monocular.git
cd ../
catkin_make
```
# How to use
will be released after it has been organized


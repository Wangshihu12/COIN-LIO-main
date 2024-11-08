# Complementary Intensity-Augmented LiDAR Inertial Odometry


<p align="center">
  <img width=125 src="doc/coin_transparent.gif">
</p>

<p align="center">
<img src="https://github.com/ethz-asl/COIN-LIO/actions/workflows/build_test_20.yml/badge.svg">
<a href="https://github.com/ethz-asl/COIN-LIO/tree/main"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
<a href="https://arxiv.org/pdf/2310.01235"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
<a href="https://arxiv.org/abs/2310.01235"><img src="https://img.shields.io/badge/arXiv-2310.01235-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
<a href="https://www.youtube.com/watch?v=H_sPLofuHpk"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
</p>

<p align="center">
  <img width='100%' src="doc/coin_tracking.gif">
</p>

<details>
<summary>Abstract</summary>
<br>
We present COIN-LIO, a LiDAR Inertial Odometry pipeline that tightly couples information from LiDAR intensity with geometry-based point cloud registration. The focus of our work is to improve the robustness of LiDAR-inertial odometry in geometrically degenerate scenarios, like tunnels or flat fields. We project LiDAR intensity returns into an intensity image, and propose an image processing pipeline that produces filtered images with improved brightness consistency within the image as well as across different scenes. To effectively leverage intensity as an additional modality, we present a novel feature selection scheme that detects uninformative directions in the point cloud registration and explicitly selects patches with complementary image information. Photometric error minimization in the image patches is then fused with inertial measurements and point-to-plane registration in an iterated Extended Kalman Filter. The proposed approach improves accuracy and robustness on a public dataset. We additionally publish a new dataset, that captures five real-world environments in challenging, geometrically degenerate scenes. By using the additional photometric information, our approach shows drastically improved robustness against geometric degeneracy in environments where all compared baseline approaches fail.
</details>

Please cite our work if you are using COIN-LIO in your research.
  ```bibtex
@inproceedings{pfreundschuh2024coin,
  title={COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry},
  author={Pfreundschuh, Patrick and Oleynikova, Helen and Cadena, Cesar and Siegwart, Roland and Andersson, Olov},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1730--1737},
  year={2024},
  organization={IEEE}
}
  ```

# Setup
## Installation
This package was developed on Ubuntu 20.04 using ROS Noetic. Other versions should also work but have not been tested and we do not guarantee support.

1. If not done yet, please [install ROS](https://wiki.ros.org/noetic/Installation), [install the proposed system dependencies](https://wiki.ros.org/noetic/Installation/Ubuntu#Dependencies_for_building_packages).
  Install some additional system dependencies:
    ```bash
    sudo apt-get install python3-catkin-tools libgoogle-glog-dev
    ```
2. Then create a catkin workspace:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin config --extend /opt/ros/$ROS_DISTRO
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    catkin config --merge-devel
    ```
3. Clone COIN-LIO into your workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone git@github.com:ethz-asl/coin-lio.git
    cd COIN-LIO
    ```
4. Build COIN-LIO:
    ```bash
    catkin build coin_lio --force-cmake
    ```

## Alternative Installation: Docker
To instead use docker, check out the repository locally, navigate to it, and:
    ```bash
    cd docker/
    ./run_docker.sh -b
    ```
Which will build a docker image with a copy of the code checked out inside.
Your `~/data` folder will be mounted to `/root/data` within the docker, so you
can download datasets and follow the rest of the tutorial below. On future runs,
you can simply use `./run_docker.sh` (without `-b`) to not re-build the image.

## Running ENWIDE Dataset Sequences
The ENWIDE dataset sequences can be downloaded [here](https://projects.asl.ethz.ch/datasets/enwide).
Run a sequence:
  ```bash
  roslaunch coin_lio mapping_enwide.launch bag_file:=<example_bag_path.bag>
  ```
## Running Newer College Dataset Sequences
The Newer College Dataset sequences can be downloaded [here](https://drive.google.com/drive/u/0/folders/1uR476FzjN3PfAiCknVKtuZi3_QfVvSdA).
Run a sequence:
  ```bash
  roslaunch coin_lio mapping_newer_college.launch bag_file:=<example_bag_path.bag>
  ```
## Running COIN-LIO on your own data:
**Note on LiDAR type:** COIN-LIO currently only supports data from Ouster LiDARs, as we use the calibration in the metadata file for the image projection model. Implementing different sensors is theoretically possible but requires a proper implementation of a projection model that works for the specific sensor. Contributions are welcome.

### Sensor Calibration
* **LiDAR:**
Since different Ouster sensors have different image projection parameters, we need to run a calibration tool to evaluate the column shift which is required to correct the image projection model. This procedure is only required once per sensor.

It is important to use the metadata file that corresponds to your specific sensor (more information can be found [here](https://github.com/ouster-lidar/ouster-ros/wiki/index)).
  ```bash
  roslaunch coin_lio calibrate.launch bag_file:=<bag_path.bag> metadata_file:=<metadata_path.json> point_topic:=<pointcloud_topic>
  ```
  The evaluated column shift parameter will be printed at the end of the procedure.
* **IMU:**
If you are not using the built-in IMU in the Ouster LiDAR, you need to adapt the extrinsic calibration between IMU and LiDAR accordingly in the [parameter file]().
### Run COIN-LIO with your own data
Launch with settings for your data:
  ```bash
  roslaunch coin_lio mapping.launch metadata_file:=<metadata_path.json> column_shift:=<parameter from calibration> point_topic:=<pointcloud_topic> imu_topic:=<imu_topic> destagger:=<true/false>
  ```
  If your data already contains [destaggered point clouds from the ouster driver](https://github.com/ouster-lidar/ouster-ros/blob/55519ed2b8a7dd7d4ae13a968b0ec88e5cada7dd/launch/common.launch#L45), set `destagger:=false`, otherwise use `destagger:=true`.
  

Play your data:
  ```bash
  rosbag play <bag_path.bag>
  ```
### Line Artifact Removal
The line artifact removal filter can be tested and tuned using the provided notebook:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/ethz-asl/COIN-LIO/blob/main/scripts/artifact_removal.ipynb)
## Acknowledgements
COIN-LIO builds on top of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) for the point-to-plane registration. 
Our dashboard was inspired by [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry). We thank the authors for open-sourcing their outstanding works.
<p align="left">
  <img width='50%' src="doc/coin_dashboard.gif">
</p>

We used [ascii-image-converter](https://github.com/TheZoraiz/ascii-image-converter) for our ascii animation.

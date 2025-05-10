# RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry
[**YouTube**](https://youtu.be/3-xLRRT25ys) | **[arXiv](https://arxiv.org/abs/2504.11580)** | **[Website](https://asig-x.github.io/resple_web/)**

This is the offcial repository for RESPLE, the first B-spline-based recursive state estimation framework for estimating 6-DoF dynamic motions. Using RESPLE as the estimation backbone, we developed a unified suite of direct LiDAR-based odometry systems, including:
* LiDAR-only odometry (LO)
* LiDAR-inertial odometry (LIO)
* Multi-LiDAR odometry (MLO)
* Multi-LiDAR-inertial Odometry (MLIO)

These four variants have been tested in real-world datasets and our own experiments, covering aerial, wheeled, legged, and wearable platforms operating in indoor, urban, wild environments with diverse LiDAR types. We look forward to your comments and feedback! 

### BibTex Citation
```
@ARTICLE{cao2025resple,
  author={Cao, Ziyu and Talbot, William and Li, Kailai},
  title={RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry}, 
  journal={arXiv preprint arXiv:2504.11580},
  year={2025}
}
``` 
### Dependencies
Tested with [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04
```
sudo apt install libomp-dev libpcl-dev libeigen3-dev
sudo apt install ros-humble-pcl*
# Optional: sudo apt install ros-humble-rosbag2-storage-mcap (for playing .mcap file if testing GrandTour dataset)
```


### Compilation
```
cd ~/ros2_ws/src
git clone --recursive git@github.com:ASIG-X/RESPLE.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select estimate_msgs livox_ros_driver livox_interfaces livox_ros_driver2 resple
```

## Docker Build

To build a docker image capable of running the examples and dataset:

```bash
cd ~/path/to/src
git clone --recursive git@github.com:ASIG-X/RESPLE.git
cd RESPLE
docker build --ssh default --tag resple .
```

## Own experimental datasets ([LINK to SURFdrive](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l)) 
Password: RESPLE2025

<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="left">
<img src="doc/hemdyn_clip.gif" width=49.6% />
<img src="doc/Rcampus_clip.gif" width = 49.6% >
</div>
<br>

**HelmDyn (Helm Dynamic) dataset**
* 1 Livox Mid360 mounted on a helmet as a mobile platform
* 10 sequences recorded with very dynamic motions combining walking, running, jumping, and in-hand waving within a cubic space   
* Ground truth trajectory recorded using a high-precision (submillimeter), low-latency motion capture system (Qualisys) invovling 20 cameras

**R-Campus dataset**
* 1 Livox Avia mounted on a bipedal wheeled robot (Direct Drive DIABLO)
* 1 sequence in walking speed recorded in a large-scale campus environment
* Trajectory starts and ends at the same location point. 


## Usage
For LIO use, change `if_lidar_only` in `resple/config/config_xxx.yaml` to `false`. 

* [HelmDyn](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l) dataset (Livox Mid360)
```
source install/setup.bash
ros2 launch resple resple_helmdyn01.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
* [R-Campus](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l) dataset (Livox Avia)

```
source install/setup.bash
ros2 launch resple resple_r_campus.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/) dataset (OUSTER OS1-16)
```
source install/setup.bash
ros2 launch resple resple_eee_02.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* [MCD](https://mcdviral.github.io/) dataset (Livox Mid70)
```
source install/setup.bash
ros2 launch resple resple_ntu_day_01.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
  
* GrandTour (Hesai XT32, Livox Mid360)
```
source install/setup.bash
ros2 launch resple resple_heap_testsite_hoenggerberg.launch.py
# ros2 launch resple resple_jungfraujoch_tunnel_small.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/hesai_livox_ap20_converted.mcap
```

### Docker

With the docker image built (see docker build instructions), one can run the run the algorithm in a docker container by following these steps.

Allow the docker user to generate graphics:
```bash
xhost +local:docker
```

Replacing `/path/to/data` with the location of the datasets, run the container (with mounted source code for development):
```bash
docker run -it -e DISPLAY=$DISPLAY \
  -v .:/root/ros2_ws/src/RESPLE \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v ~/data/resple_dataset/:/root/data/resple_dataset \
  -v ~/data/grand_tour_box/datasets:/root/data/grand_tour_box/datasets \
  --name resple resple
```
Note: To recompile inside the docker container run `colcon build --packages-up-to resple`. If no development is intended, then one can omit `-v .:/root/ros2_ws/src/RESPLE`.

Replacing `<filename>` with the launch file from above, launch with:
```bash
ros2 launch resple <filename>.launch.py
```

Create a second terminal attached to the container with:
```bash
docker exec -it resple bash
```

In this second container, replacing `<example>/<filename>` to make a valid bag filepath, play the dataset:
```bash
ros2 bag play ~/data/resple_dataset/<example>/
```

If the container is already run, then:
* It can be removed with:
```bash
docker rm resple
```
* It can be started with:
```bash
docker start resple
```
* It can be be attached to with:
```bash
docker attach resple
```
* It can be stopped with:
```bash
docker stop resple
```

## Contributors
Ziyu Cao (Email: ziyu.cao@liu.se)

William Talbot (Email: wtalbot@ethz.ch)

Kailai Li (Email: kailai.li@rug.nl)

## Credits
Thanks for [SFUISE](https://github.com/ASIG-X/SFUISE), [ikd-Tree](https://github.com/hku-mars/ikd-Tree), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [Livox-SDK](https://github.com/Livox-SDK), and [basalt](https://gitlab.com/VladyslavUsenko/basalt).

## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

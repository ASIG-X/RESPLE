# RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry
[**YouTube**](https://youtu.be/3-xLRRT25ys) | **[arXiv](https://arxiv.org/abs/2504.11580)** | **[Website](https://asig-x.github.io/resple_web/)** | **[IEEE RA-L](https://doi.org/10.1109/LRA.2025.3604758)** | **[Demonstrator](https://asig-x.github.io/resple_demonstrator/)**

--> [Branch for benchmarking](https://github.com/ASIG-X/RESPLE/tree/feature/benchmark)
### News
* 2026-01: A new feature is now available to save estimated trajectories specifically for benchmarking. Please check out the branch [here](https://github.com/ASIG-X/RESPLE/tree/feature/benchmark). An example using the HelmDyn dataset can be found [here](https://github.com/ASIG-X/RESPLE/blob/feature/benchmark/resple/config/config_helmdyn01.yaml). The estimated trajectory will be generated through spline interpolation at ground-truth timestamps in a `.txt` file following the [TUM format](https://github.com/MichaelGrupp/evo/wiki/Formats).
* 2025-12: The design of a handheld demostrator for RESPLE is now publicly available. Check out our web page [here](https://asig-x.github.io/resple_demonstrator/).
* 2025-12: Additional evaluation results of RESPLE-LIO and corresponding parameter sets on the [Newer College](https://ori-drs.github.io/newer-college-dataset/) dataset (including its extension) and the [MCD](https://mcdviral.github.io/) dataset are now available on our [web page](https://asig-x.github.io/resple_web/add_evaluation.html). Instructions for testing are given below.
* 2025-09: The [TudoRun](https://asig-x.github.io/resple_web/datasets.html) dataset is released as a supplementary dataset.


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
  journal={IEEE Robotics and Automation Letters},
  volume={10},
  number={10},
  pages={10666-10673},
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
git clone --recursive https://github.com/ASIG-X/RESPLE.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select estimate_msgs livox_ros_driver livox_interfaces livox_ros_driver2 resple
```

## Docker Build

To build a docker image capable of running the examples and dataset:

```bash
cd ~/path/to/src
git clone --recursive https://github.com/ASIG-X/RESPLE.git
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
* Ground truth trajectory recorded using a high-precision (submillimeter), low-latency motion capture system (Qualisys) involving 20 cameras

**R-Campus dataset**
* 1 Livox Avia mounted on a bipedal wheeled robot (Direct Drive DIABLO)
* 1 sequence in walking speed recorded in a large-scale campus environment
* Trajectory starts and ends at the same location point. 

**TudoRun (Tudor Run) dataset**
* 1 Livox Mid360 mounted on a Unitree Go2 quadruped robot
* 8 indoor sequences with dynamic motions: 3 fully captured in a test field with an 8-camera motion capture system, and 5 starting and ending in the test field but extending into a larger hall without motion capture
* Ground truth trajectory recorded only within this test field using the motion capture system (Qualisys) with passive markers

**Please refer to our [dataset website](https://asig-x.github.io/resple_web/datasets.html) for more information.**
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
* [TudoRun](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l) dataset (Livox Mid360)

```
source install/setup.bash
ros2 launch resple resple_tudorun01.launch.py
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
ros2 launch resple resple_ntu_day_01_livox.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
  
* [GrandTour](https://grand-tour.leggedrobotics.com/) dataset (Hesai XT32, Livox Mid360)
```
source install/setup.bash
ros2 launch resple resple_heap_testsite_hoenggerberg.launch.py
# ros2 launch resple resple_jungfraujoch_tunnel_small.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/hesai_livox_ap20_converted.mcap
```

* [Newer College](https://ori-drs.github.io/newer-college-dataset/stereo-cam/) (OUSTER OS1-64)
```
source install/setup.bash
ros2 launch resple resple_nc_short.launch.py
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* [Extension to Newer College](https://ori-drs.github.io/newer-college-dataset/multi-cam/) (OUSTER OS0-128)
```
source install/setup.bash
ros2 launch resple resple_nce_quad.launch.py # for outdoor sequences
# ros2 launch resple resple_nce_stairs.launch.py # for indoor sequences Stairs and Mine-*
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```

* [MCD](https://mcdviral.github.io/) dataset (OUSTER OS1-64-VN200 or OS1-128-VN100)
```
source install/setup.bash
ros2 launch resple resple_ntu_day_01_ouster.launch.py # for sequences ntu_*
# ros2 launch resple resple_kth_day_06_ouster.launch.py # for sequences kth_* and tuhh_*
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
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

## Get Started

- Install [ROS](http://www.ros.org/install/).

- Install [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).

- Install [ROS Navigation stack](http://wiki.ros.org/navigation). You can install it by running ```sudo apt-get install ros-indigo-navigation```. If you are using other versions of ROS, replace indigo in the command with your ROS version.


## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clong https://github.com/TixiaoShan/jackal_mapping.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## Run the System (in simulation)

1. Run the launch file:
```
roslaunch tm_mapping offline.launch
```

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
Notes: our system only needs /velodyne_points for input from bag files. However, a 3D SLAM method usually needs /imu/data.

## Run the System (with real robot)

Run the launch file:
```
roslaunch tm_mapping online.launch
```

Note: if you are using other nodes that publishes tf transform for base_link, you may want to stop it. For example, run ```rosnode kill /ekf_localization``` to kill robot_localization node.

## Cite *tm_mapping*

Thank you for citing our paper if you use any of this code: 
```
@inproceedings{traversability2018,
  title={Bayesian Generalized Kernel Inference for Terrain Traversability Mapping},
  author={Tixiao Shan, Kevin Doherty, Jinkun Wang and Brendan Englot},
  booktitle={In Proceedings of the 2nd Annual Conference on Robot Learning},
  year={2018}
}
```

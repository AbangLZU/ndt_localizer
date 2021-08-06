## A clean and simple NDT localizer

This repo modified from Autoware lidar_localizer module. Unlike the module in Autoware with haveily dependency on a lot of packages(you need compile all the packages in Autoware project), this repo is clean, simple and with no dependencies. All you need is ROS, and a pcd file(the point cloud map). 

Let's start our lidar-based localization learning with this simple repo!


## Localization in a pointcloud map(pcd)
![](cfgs/ndt_result.gif)

A demo video on MulRan dataset:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/qhqDmmO7c4c/0.jpg)](https://www.youtube.com/watch?v=qhqDmmO7c4c)

## How to use

### Prepare you pcd map and rosbag

You can reproduce my blog [基于NDT的自动驾驶高精度定位和ROS项目实战](https://blog.csdn.net/AdamShan/article/details/106739856?spm=1001.2014.3001.5501) and [使用SC-LEGO-LOAM进行较大规模点云地图构建和闭环优化](https://blog.csdn.net/AdamShan/article/details/106589633?spm=1001.2014.3001.5501) to use Mulran dataset to build your pcd map and produce the pointcloud data. Unfortunately, the blog is written with Chinese, if you can not read Chinese blog and want to reproduce the project demo, use the link below(Baidu disk) to download the pcd map and rosbag:

link: https://pan.baidu.com/s/1hZ0VuQCy4KX3lHUTFdVeww  passward: r7fl

![](cfgs/4.png)

> The KAIST02-small.bag is not the whole KAIST02 dataset, because the rosbag do not compress data, the whole KAIST02 rosbag is too large. So I use the first 81 seconds of the KAIST02 dataset to make this small rosbag.

Put the pcd data to the map folder:

```bash
cp kaist02.pcd map/
```

### Build in your ros workspace
clone this repo in your `ros workspace/src/`, and then `catkin_make` (or `catkin build`):
```bash
cd catkin_ws/src/
git clone https://github.com/AbangLZU/ndt_localizer.git
cd ..
catkin_make
```

### Setup configuration

#### Config map loader
Move your map pcd file (.pcd) to the map folder inside this project (`ndt_localizer/map`), change the pcd_path in `map_loader.launch` to you pcd path, for example:

```xml
<arg name="pcd_path"  default="$(find ndt_localizer)/map/kaist02.pcd"/>
```
#### Config point cloud downsample

Config your Lidar point cloud topic in `launch/points_downsample.launch`:

```xml
<arg name="points_topic" default="/os1_points" />
```

If your Lidar data is sparse (like VLP-16), you need to config smaller `leaf_size` in `launch/points_downsample.launch` like `2.0`. If your lidar point cloud is dense (VLP-32, Hesai Pander40P, HDL-64 ect.), keep `leaf_size` as `3.0`。

#### Config static tf

There are two static transform in this project: `base_link_to_localizer` and `world_to_map`，replace the `ouster` with your lidar frame id if you are using a different lidar:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_localizer" args="0 0 0 0 0 0 base_link ouster"/>
```

#### Config ndt localizer
You can config NDT params in `ndt_localizer.launch`. Tha main params of NDT algorithm is:

```xml
<arg name="trans_epsilon" default="0.05" doc="The maximum difference between two consecutive transformations in order to consider convergence" />
<arg name="step_size" default="0.1" doc="The newton line search maximum step length" />
<arg name="resolution" default="2.0" doc="The ND voxel grid resolution" />
<arg name="max_iterations" default="30.0" doc="The number of iterations required to calculate alignment" />
<arg name="converged_param_transform_probability" default="3.0" doc="" />
```

These default params work nice with 64 and 32 lidar.

### Run the localizer
Once you get your pcd map and configuration ready, run the localizer with:


```bash
# open a roscore
roscore
# in other terminal
cd catkin_ws
source devel/setup.bash
# use rosbag sim time if you are playing a rosbag!!!
rosparam set use_sim_time true
# launch the ndt_localizer node
roslaunch ndt_localizer ndt_localizer.launch
```

wait a few seconds for loading map, then you can see your pcd map in rviz like this:

![](cfgs/sample_img_1.png)

give a init pose of current vehicle with 2D Pose Estimate in the rviz:

![](cfgs/sample_img3.png)


This operation will send a init pose to topic `/initialpose`.

play the rosbag:

```bash
rosbag play KAIST02-small.bag --clock
```

Then you will see the localization result:

![](cfgs/sample_img2.png)

The final localization msg will send to `/ndt_pose` topic:

```proto
---
header: 
  seq: 1867
  stamp: 
    secs: 1566536121
    nsecs: 251423898
  frame_id: "map"
pose: 
  position: 
    x: -94.8022766113
    y: 544.097351074
    z: 42.5747337341
  orientation: 
    x: 0.0243843578881
    y: 0.0533175268768
    z: -0.702325920272
    w: 0.709437048124
---
```

The localizer also publish a tf of `base_link` to `map`:

```
---
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1566536121
        nsecs: 251423898
      frame_id: "map"
    child_frame_id: "base_link"
    transform: 
      translation: 
        x: -94.8022766113
        y: 544.097351074
        z: 42.5747337341
      rotation: 
        x: 0.0243843578881
        y: 0.0533175268768
        z: -0.702325920272
        w: 0.709437048124
```


### Want to know more detail?
You can follow my blog series in CSDN (Chinese): https://blog.csdn.net/adamshan

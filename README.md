# First Assignment for the course "Sofwtare Architectures for Robotics" - UniGe
### Developed by  [@LoreBene99](https://github.com/LoreBene99), [@andreamanera](https://github.com/andreamanera), [@dssdanial](https://github.com/dssdanial)  
Assignment given by the professors Simone Macciò and Fulvio Mastrogiovanni for the course "Sofwtare architectures for robotics" - Robotics Engineering, at the University of Genova.

## Delivery of the project 
<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/Immagine.png" width="550" height="400">
</p>

## Preface

**Slam Toolbox** is a set of tools and capabilities for 2D SLAM built by *Steve Macenski*.
This project contains the ability to do most everything any other available SLAM library, both free and paid, and more. This includes :

* Ordinary point-and-shoot 2D SLAM mobile robotics folks expect (start, map, save pgm file) with some nice built in utilities like saving maps.
* Continuing to refine, remap, or continue mapping a saved (serialized) pose-graph at any time.
* Lifelong mapping: load a saved pose-graph continue mapping in a space while also removing extraneous information from newly added scans.
* An optimization-based localization mode built on the pose-graph. Optionally run localization mode without a prior map for **lidar odometry** mode with local loop * closures.
* Synchronous and asynchronous modes of mapping.
* Kinematic map merging (with an elastic graph manipulation merging technique in the works).
* Plugin-based optimization solvers with a new optimized Google Ceres based plugin.
* RVIZ plugin for interacting with the tools.
* Graph manipulation tools in RVIZ to manipulate nodes and connections during mapping.
* Map serialization and lossless data storage.
* More...

Whereas **Nav2** project is the spiritual successor of the ROS Navigation Stack. This project seeks to find a safe way to have a mobile robot move from point A to point B. It can also be applied in other applications that involve robot navigation, like following dynamic points. This will complete dynamic path planning, compute velocities for motors, avoid obstacles, and structure recovery behaviors. To learn more about this project, such as related projects, robots using, ROS1 comparison, and maintainers, see About and Contact.

Nav2 uses behavior trees to call modular servers to complete an action. An action can be to compute a path, control effort, recovery, or any other navigation related action. These are each separate nodes that communicate with the behavior tree (BT) over a ROS action server. The diagram below will give you a good first-look at the structure of Nav2. 

<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/nav2_architecture.png" width="550" height="400">
</p>

## Installation 
First things first the entire project works on the **Ros galactic distribution**. 

Since we have to test and explore the capabilities of new localization and mapping framework with different map layouts, using **Nav2 package** for robot navigation and **SLAM Toolbox** for 2D mapping, you have to install them :
*  **Installing SLAM toolbox** : 
```
sudo apt install ros-galactic-slam-toolbox
```
*  **Installing Nav2**: 
```
sudo apt-get install ros-galactic-navigation2
```
*  **Installing Nav2-bringup**: 
```
sudo apt-get install ros-galactic-nav2-bringup
```
Then you have to update the dependencies using this particular command:
```
rosdep update 
```
Exactly in the **ROOT** folder of your workspace, run this command : 
```
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO (galactic)
```

In the end run : 
```
colcon build 
``` 
to build the package.

#### NB : we decided to not remove the subfolder in webots_ros_2 called *webots_ros2_core* even if this package has been deprecated and will be removed with the release of Webots R2023a. In fact, users of webots_ros2_core should migrate to *webots_ros2_driver*.

## Run the project
To run the project you will need your own workspace, in which you have to clone this repository in the *src* folder.

Then run on the terminal *colcon build* in order to set up the package. 

### Launch Arguments
In order to have "a better view" of the project, we decided to write all the launch file arguments, to not have a confusionary "vision" of the whole project.
* **nav** : to run the simulation using the navigation stack.
* **slam**: to use slam.
* **lifelong**: to use the lifelong modality of the slam_toolbox.
* **localization** : to use the slam toolbox ony in localization mode.
* **use_sim_time** : to use the simulation time (default : True)
* **world** : to choose the environment for the simulation (default : *break_room_1.wbt* file in the **environments** folder; if you'd like to change the 3D environments, you have to put, in the row launch code on the terminal, *break_room_2.wbt* / *warehouse.wbt* / *house.wbt*.  
* **map** : to choose the map for the navigation (map.yaml file as default)
* **mode** : choose the simulator mode (*realtime* as deafault) 
* **nav_params** : to choose the parameters for navigation (nav2_parameters.yaml as default)
* **slam_params** : to choose the parameters for slam mapping (online_async_parameters.yaml as default)
* **rviz** : in order to use rviz.

### Nav 2 mode
As we said before, the Nav2 project is the spiritual successor of the ROS Navigation Stack. This project seeks to find a safe way to have a mobile robot move from point A to point B. It can also be applied in other applications that involve robot navigation, like following dynamic points. This will complete dynamic path planning, compute velocities for motors, avoid obstacles, and structure recovery behaviors. 

In order to launch the simulation using the navigation stack, you have to run this command :
```
ros2 launch tiago_assignment assignment_launch.py nav:=true rviz:=true
```
After that, to start correctly the simulation, you must place the robot through the utilization of **2D Pose Estimate**; to let the robot reach the goal, use **2D Nav Goal** (always in  *rviz*) and select wherever you want the robot to go.
#### NB : In this modality, the pre-loaded map is a map from the folder "resource" (comes from the "tiago" subfolder in the webots_ros2 package). Of course we have to precisize that the 2D map has to MATCH with the 3D environment, if not there will be some mismatches! This map is matching with the break_room_1 (default).

https://user-images.githubusercontent.com/91314586/178226028-71e5dacc-6215-40c4-8630-9c1344591cb3.mp4

### Slam (Simultaneous Localization and Mapping) mode
The Slam Toolbox package incorporates information from **laser scanners** in the form of a **LaserScan message** and TF transforms from odom->base link, and **creates a 2D map of a space**.
This package will allow you to **fully serialize the data and pose-graph of the SLAM map** to be reloaded to continue mapping, localize, merge, or otherwise manipulate. We allow for SLAM Toolbox to be run in synchronous (process all valid sensor measurements, regardless of lag) and asynchronous (process valid sensors measurements on an as-possible basis) modes.

In order to launch the simulation using only **slam** you have to run this command :
```
ros2 launch tiago_assignment assignment_launch.py slam:=true rviz:=true
``` 
Then, if you want to move the robot in the presented environment (which you can change) using the keyboard, run this command on another terminal :
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
``` 
At this point the robot start to create the map (you can see it on **rviz**), that you can save through this command : 
```
ros2 run nav2_map_server map_saver_cli -f ~/map
``` 
It seems reasonable the fact that you can also use the **Nav2 Goal** command on rviz to guide the robot in the environment. 

https://user-images.githubusercontent.com/91314586/178226220-55828ced-a93c-49ed-8e15-4b66ccf5dcc7.mp4

### Localization mode
Localization mode consists of 3 things:
* Loads existing serialized map into the node.
* Maintains a rolling buffer of recent scans in the pose-graph.
* After expiring from the buffer scans are removed and the underlying map is not affected. 

When the map has been serialized, you will have to add its FULL path to the *localization_parameters.yaml* file, subsequently you have to run :
```
ros2 launch tiago_assignment assignment_launch.py localization:=true rviz:=true
```
In this case you will use the **slam_toolbox** package only as localization, of course using the map file.
This will allow you to use the slam_toolbox package as localization only using your map file.
#### NB : For this modality there is an example map (not a completed one) in the *maps* folder, called "house" (house.data/house.posegraph). You can start the Localization mode using that map, putting the FULL path to that map in the localization_parameters.yaml. The 3D environments is the house, so put on the terminal *world:=house.wbt*.

https://user-images.githubusercontent.com/91314586/178226291-67ccb770-a856-4d26-947b-f334f6941e5f.mp4

### LifeLong mapping mode
LifeLong mapping is the concept of being able to map a space, completely or partially, and over time, refine and update that map as you continue to interact with the space. While Slam Toolbox can also just be used for a point-and-shoot mapping of a space and saving that map as a .pgm file as maps are traditionally stored in, it also allows you to save the pose-graph and metadata losslessly to reload later with the same or different robot and continue to map the space.

Our lifelong mapping consists of a few key steps :
* Serialization and Deserialization to store and reload map information
* KD-Tree search matching to locate the robot in its position on reinitalization
* pose-graph optimizition based SLAM with 2D scan matching abstraction

This will allow the user to create and update existing maps, then serialize the data for use in other mapping sessions, something sorely lacking from most SLAM implementations and nearly all planar SLAM implementations.
You should save the map and its relative posegraph when the session ends. The map file name should be updated in lifelong_parameters. 

If you want to run the project, you have to insert this command : 
```
ros2 launch tiago_assignment assignment_launch.py lifelong:=true rviz:=true
```
You can also desiarialize the pose graph and continue mapping the world (through **rviz**).
Remember : In the Deserialize **box**, in rviz, you have to put the full path of the map you want to deserialize; in this case, the map you want to continue mapping will be loaded (ex : */home/lorenzo/sofar_ws/src/SofAR_Assignment1/tiago_assignment/maps/half_map*), ready to be mapped (again)! Of course we have tested the LifeLong mode and we have generated the related after_lifelong files in the *maps* folder regarding the continue mapping on this map. The half_map was built in the **warehouse** environment, so put world:=warehouse.wbt when starting this modality. 

https://user-images.githubusercontent.com/91314586/178328555-c6d99885-bf57-4c0b-8e43-099e76db637d.mp4    

## Nodes Interaction
### SLAM mode without the keyboard

<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/slamnokey.png" width="550" height="400">
</p>

### SLAM mode with the keyboard

<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/slamkey.png" width="550" height="400">
</p>

### Nav2 mode 

<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/nav.png" width="550" height="400">
</p>

### Localization mode

<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/localization.png" width="550" height="400">
</p>

### LifeLong mode

<p align="center">
<img src="https://github.com/LoreBene99/SofAR_Assignment1/blob/main/images/lifelong.png" width="550" height="400">
</p>

## Conclusion 
The assignement was very interesting and exciting to develop.
We managed to deal with the **slam_toolbox** and **Nav2** thanks to all the documentation provided on Github and Internet.
Especially we took inspiration from the Slam_toolbox packaged provided by *Steve Macenski* on Github and from the *ros-planning/navigation2* package.
We also took advantage of the *cyberbotics/webots_ros2* in order to drive easly TIAgo in the appropriate 3D environments built purposely to test its SLAM skills.
You can find the presentation of the project [here](https://github.com/LoreBene99/SofAR_Assignment1/blob/main/group13_sofar_presentation.pptx).

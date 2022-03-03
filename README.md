# P5-Home-Service-Robot
Udacity Robotics Software Engineer Nanodegree 

## Overview  
In this project, you will use everything you learned in the Nanodegree Program to build a Home Service Robot in ROS.
### Shell Scripts
A shell script is a file containing a series of commands and could be executed. It is commonly used to set up environment, run a program.
You will create a launch.sh script and use it to launch Gazebo and Rviz in separate instances of terminals.
Steps
- Install xterm with `sudo apt-get install xterm`
- Create launch.sh file
- Write following lines in to launch.sh file
![image](https://user-images.githubusercontent.com/15081906/156510083-37491a7f-deea-4ab0-a944-ac870853f4dc.png)
- Turn script ino an executable one with `chomd +x launch.sh`
- Test script file with `./launch.sh`
### SLAM Testing 
The goal of this step is to manually test SLAM. You will write a shell script `test_slam.sh` that will deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz. You can find the file `test_slam.sh` in the folder `scripts`.  
Steps
- Create `test_slam.sh`
- Change terminal directory to `scripts`. Then, launch `test_slam.sh` by terminal command `./test_slam.sh` 
### Localization and Navigation Testing
The task of this project is to pick two different goals and test your robot's ability to reach them and orient itself with respect to them. You will be using the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan our robot trajectory from start to goal position. 
Steps
- Create a `test_navigation.sh` script file to launch it for manual navigation test.  
- Change terminal directory to `scripts`. Then, launch `test_slam.sh` by terminal command `./test_slam.sh`
- After Rviz is launched, you can use `2D Nav Goal` to manually point out to two different goals, one at a time, and direct your robot to reach them and orient itself with respect to them.
![image](https://user-images.githubusercontent.com/15081906/156513107-f69914d7-6ad8-4b01-8efa-e3a0fe925787.png)
### Navigation Goal Node
You will write a node that will communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach. 
Steps
- Create ros package `pick_objects` by command `catkin_create_pkg pick_objects move_base_msgs actionlib roscpp`
- Create .cpp file `pick_objects` and modified the code. You can find `pick_objects.cpp` in the folder `pick_objects/src/`
- Edit the `CMakeLists.txt` file and add `directories`, `executable`, and `target_link_libraries`. You can find `CMakeLists.txt` in `pick_objects/`
- Create a `pick_objects.sh` file that will send multiple goals for the robot to reach. You can find this file in folder `scripts`
- Switch terminal directory to `catkin_ws/`. Then, build `catkin_ws` by terminal command `catkin_make` 
- Launch `pick_objects.sh` by command `./pick_objects.sh`
![image](https://user-images.githubusercontent.com/15081906/156515759-044fc68b-f72f-4d99-8660-e6aac6f7e8b0.png)
### Home Service Robot
In this project, you will write an add_marker node that subscribe to your odometry to keep track of your robot pose. When the robot reaches pick-up zone, the add_marker node will delete the marker in Rviz, and when the robot reaches drop-off zone, the add_marker node will show marker again in Rviz.
Step
- Create ros package `add_markers` by terminal command `catkin_create_pkg add_markers visualization_msgs roscpp`
- Create .cpp file `add_markers` and modified the code. You can find `add_markers.cpp` in the folder `ad_markers/src/`
- Edit the `CMakeLists.txt` file and add `executable`, and `target_link_libraries`. You can find `CMakeLists.txt` in `add_markers/`
- Create a `home_service.sh` file that will send multiple goals for the robot to reach. You can find this file in folder `scripts`
- Switch terminal directory to `catkin_ws/`. Then, build `catkin_ws` by terminal command `catkin_make` 
- Launch `home_service.sh` by command `./home_service.sh`. In Rviz, you will see your robot is going to pick up the marker.
![image](https://user-images.githubusercontent.com/15081906/156518475-f1d4a6f9-0688-4492-99b2-00855bae386b.png)
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line and execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. On the command line and execute  
```
cd RoboND-Term1-P5-Home-Service-Robot/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
5. Build and run your code.  

## Project Description  
Directory Structure  
```
├── catkin_ws                                             # Catkin workspace
│   ├── src
│   │   ├── add_markers                                   # add_markers package        
│   │   │   ├── launch
│   │   │   │   ├── view_home_service_navigation.launch   # launch file for home service robot demo
│   │   │   ├── src
│   │   │   │   ├── add_markers.cpp                       # source code for add_markers node
│   │   │   │   ├── add_markers_demo.cpp                  # source code for add_markers_demo
│   │   ├── pick_objects                                  # pick_objects package     
│   │   │   ├── src
│   │   │   │   ├── pick_objects.cpp                      # source code for pick_objects node
│   │   │   │   ├── pick_objects_demo.cpp                 # source code for pick_objects_demo
│   │   ├── rvizConfig                                    # rvizConfig package        
│   │   │   ├── home_service_rvizConfig.rviz              # rvizConfig file for home service robot demo  
│   │   ├── scripts                                       # shell scripts files
│   │   │   ├── add_marker.sh                             # shell script to model virtual objects  
│   │   │   ├── home_service.sh                           # shell script to launch home service robot demo  
│   │   │   ├── pick_objects.sh                           # shell script to send multiple goals  
│   │   │   ├── test_navigation.sh                        # shell script to test localization and navigation
│   │   │   ├── test_slam.sh                              # shell script to test SLAM
│   │   ├── slam_gmapping                                 # gmapping_demo.launch file
│   │   ├── turtlebot                                     # keyboard_teleop.launch file
│   │   ├── turtlebot_interactions                        # view_navigation.launch file
│   │   ├── turtlebot_simulator                           # turtlebot_world.launch file package        
│   │   ├── CMakeLists.txt                                # compiler instructions
├── video.mp4                                             # Videos for overview
├── video.gif                                             # GIF for overview
```
- [view_home_service_navigation.launch](/catkin_ws/src/add_markers/launch/view_home_service_navigation.launch): Launch rviz with specify rviz configuration file  
- [add_markers.cpp](/catkin_ws/src/pick_objects/src/add_markers.cpp): C++ script, communicate with `pick_objects` node and control the marker appearance to simulate object pick up and drop off   
- [pick_objects.cpp](/catkin_ws/src/pick_objects/src/pick_objects.cpp): C++ script, communicate with `add_markers` node and command the robot to pick up the object  
- [home_service_rvizConfig.rviz](/catkin_ws/src/rvizConfig/home_service_rvizConfig.rviz): rvizConfig file for home service robot demo which contained `markers` option  
- [add_marker.sh](/catkin_ws/src/scripts/add_marker.sh): Shell script file to deploy a turtlebot inside your environment, model a virtual object with markers in `rviz`.  
- [home_service.sh](/catkin_ws/src/scripts/home_service.sh): Shell script file to deploy a turtlebot inside your environment, simulate a full home service robot capable of navigating to pick up and deliver virtual objects.  
- [pick_objects.sh](/catkin_ws/src/scripts/pick_objects.sh): Shell script file to deploy a turtlebot inside your environment, communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach.  
- [test_navigation.sh](/catkin_ws/src/scripts/test_navigation.sh): Shell script file to deploy a turtlebot inside your environment, pick two different goals and test your robot's ability to reach them and orient itself with respect to them.  
- [test_slam.sh](/catkin_ws/src/scripts/test_slam.sh): Shell script file to deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in `rviz`  

- [CMakeLists.txt](/catkin_ws/src/CMakeLists.txt): File to link the C++ code to libraries.  
- [video.gif](video.gif): A gif of video for final home service robot run  

## Run the project  
* Clone this repository
```

```
* Navigate to the `src` folder and clone the necessary repositories  
```
cd RoboND-Term1-P5-Home-Service-Robot/catkin_ws/src  
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git  
```
* Open the repository, make and source  
```
cd /home/workspace/catkin_ws/
catkin_make
source devel/setup.bash
```
* Launch the home service robot
```
./src/scripts/home_service.sh
```
* Done. 

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```
sudo apt-get update && sudo apt-get upgrade -y
```
2. If your system python version from miniconda is python3 while the ros packages and tf are python2. A hack is to just set the system python to python2 via symbol link. Run the following commands to resolve it  
```
ln -s /usr/bin/python2 /root/miniconda3/bin/python
```
3. How to setup your environment at start up.  
```
Add the following line into the /home/workspace/.student_bashrc
export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages  
pip install catkin_pkg  
pip install rospkg  
```
4. How create package with dependencies  
```
catkin_create_pkg pick_objects move_base_msgs actionlib roscpp  
catkin_create_pkg add_markers roscpp visualization_msgs  
```
5. How to visualize your marker in the rviz  
To see the marker(virtual objects) demo, in addition to running the `./add_marker.sh`, you will need to manually add a 'Marker' in rviz with the following steps:  
* Find your rviz window  
* In the left bottom panel, click "Add" button  
* In 'By display type' tab, navigate the tree to 'rviz' then 'Marker'  
* Click 'OK' button  
* Done, you should see the marker(virtual objects) appear, disappear then appear again  

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

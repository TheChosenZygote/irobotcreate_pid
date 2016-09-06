# irobotcreate_pid

This is a pid controller for waypoints for the iRobot Create using ROS. Make sure you have a package for an iRobot Create installed already before attempting to install this package. [This](https://github.com/MirkoFerrati/irobotcreate2ros) is the one I used.

## Installation

Create a catkin workspace  

```
$ mkdir -p pid_control_ws/src  
$ cd pid_control_ws  
$ catkin init  
```

In pid_control_ws/src, clone this repo  

```
$ git clone https://github.com/TheChosenZygote/irobotcreate_pid
```

Install dependencies  

```
$ rosdep update  
$ rosdep install --from-paths src -i  
```

Build  

```
$ catkin make  
```

Source  

```
$ source ~/"workspace name"/devel/setup.bash  
```

## Running

Make sure you have Gazebo running and make sure you have the iRobot spawned in the map. Then run the pid controller  

```
rosrun irobotcreate_pid pid_control_node2  
```

You can adjust, add, and remove waypoints accordingly by going into the pid_control_node2.cpp file and manipulating them. Just make sure you make the workspace after every edit.

# irobotcreate_pid

This is a pid controller for waypoints for the iRobot Create using ROS. Make sure you have a package for an iRobot Create installed already before attempting to install this package. [This](https://github.com/MirkoFerrati/irobotcreate2ros) is the one I used, and [this](https://github.com/AutonomyLab/create_autonomy) is also a good alternative.

## Installation

In the src folder of an existing catkin workspace, clone this repo
<code> $ git clone https://github.com/TheChosenZygote/irobotcreate_pid</code>

Install dependencies
<code> $ rosdep update</code>
<code> $ rosdep install --from-paths src -i</code>

Build
<code> $ catkin_build build</code>

Source
<code> $ source ~/"workspace name"/devel/setup.bash</code>

## Running

Make sure you have Gazebo running and make sure you have the iRobot spawned in the map. Then run the pid controller
<code> rosrun irobotcreate_pid_node irobotcreate_pid_node</code>

You can adjust, add, and remove waypoints accordingly by going into the irobotcreate_pid_node.cpp file and manipulating them.

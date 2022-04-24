# B31XP-G1-ERL-HSRB

## My code submitted for the B31XP - MultiDisciplinary Group Project - 2021-2022

While this was build from last years work, it can function by itself with limited features
Please check it for instructions on how to set up the environment
https://github.com/care-group/European-Robotics-League-MEng-Project-2021

Make sure you've sourced setup.bash as well as built the environment
```
cd workspace
source devel/setup.bash
catkin_make
```

Navigation stack can be executed by running
```
roslaunch nav_tests azm_nav_semantic_node.launch
```

Control node can be launched by running
```
roslaunch control ctrl.launch
```

To move to an object location publish to this topic the location
```
rostopic pub /azm/nav/semantic/goal_listener std_msgs/String "data: 'grannyAnnie'"
```

To start the example publish an object to the fetch this topic
```
rostopic pub /azm/ctrl/fetch_this std_msgs/String "data: 'shelves'"
```

If you have any questions about the semantic map which aren't answered in either of the reports feel free to contact me

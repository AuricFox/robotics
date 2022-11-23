## Robotic, ROS Language

Samuel Kitzerow, kitze012

### Running Program

Initial Launch (In Separate Terminal):  
Launching the master node.
```
roscore
```

Launching Programs:
```
python3 filename
python3 example-publisher_kitze012.py
python3 example-subscriber_kitze012.py
python3 example-controller_kitze012.py
```

Display Graph:
```
sqt_graph
```

Display Topic List:
```
rostopic list
```

### Special Note
Nothing will be displayed on the publisher terminal when the publisher node is launched. Info will be sent to the subscriber node and displayed on the subscriber terminal.

### Edit `/.bashrc` file

Use `vim ~/.bashrc` to edit environment. Check for bash file using `ls -a`.

```
source /opt/ros/noetic/setup.bash
source /path/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```
# Bimanual and single armed polishing of a car-wheel

## Installation
In your catkin src directory clone the repository
```
$ git clone https://github.com/epfl-lasa/kuka-lwr-ros.git
```
wstool gets all other git repository dependencies, after the following steps you should see extra catkin packages in your src directory.

```
wstool init
wstool merge wheel-polishing/dependencies.rosinstall 
wstool up 
```

Query and installs all libraries and packages
```
rosdep install --from-paths . --ignore-src --rosdistro indigo 
```

## Launch this simulation

### Launch UR5 robot arm
If the two arms are coordinating together, synchronize the time between the two controlling computers. (This might kill the rosmaster, so do it before starting anything.)
Next terminal, synchronize time. Check delay:
```
ntpdate -q 192.168.0.20
```

Synchronize time:
```
sudo ntpdate 192.168.0.20
```

And start the ridgeback. 

Go on UR5
```
ssh administrator@192.168.0.20
```
PW: clearpath

```
roslaunch cpr_bringup cpr_bringup.launch sim:=false
```

### Launch KUKA
The polishing robot with corresponding tool.
```
roslaunch lwr_simple_example real.launch
```
Launch the file on the KUKA Controller. Launch FRI interface:
```
roslaunch lwr_fri lwr_fri_console.launch
```
Type on the same terminal
```
control
```

Launch force torque sensors
```
roslaunch netft_rdt_driver ft_nano_sensor.launch 
```
For other force torque sensors launch different launch script. It only differs in the IP of the sensors. 


set DS controller paramters
```
rosrun rqt_reconfigure rqt_reconfigure 
```
damping_eigval0:150, damping_eigval1:150, rot_stiffness:15, rot_damping:2, rot_integrator:0, smooth_val:0.001, debug:True, bSmooth:True, useNullSpace:True, jointLImitsGains:1.0, desiredJointsGain:0.0, jointVelocitiesGain:0.01  


#### Simplified Demo (only one Robot)
To run only the simplified demo (no ridgeback)
```
rosrun polishing_demo simple_polishing_demo test -s p -v 0.2	-f 15 -an n
```
NOTE: to increase velocity change keyword argument -v [0.4]
!!! An update of the parameters should show at a frequency of about >10Hz. If this is not the case, restart the node.

#### Advanced Demo (Robot arm polishing and second one holding the wheel)
Publish calibration between KUKA and UR5
```
roslaunch robot_calibration calibration_publishers.launch ci:=1
```

Launch KUKA controller
```
rosrun polishing_demo polishing_demo bou -v 0.4 -f 12 -an n
```



Launch UR5 arm
```
rosrun wheel_polishing ur5_move_wheel_sequence.py 
```

Visualize tools
```
rosrun wheel_polishing visualization_tools.py
```


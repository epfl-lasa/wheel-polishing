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

Set DS controller paramters
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

### Basic launching of the robot
```
roslaunch roslaunch lwr_simple_example sim.launch 
rosrun rqt_reconfigure rqt_reconfigure
```
Increase values for damping_eigVal1 and damping_eigVal2 to maximum value gradually.

On the kuka interface:
Run script "nadia"
```
roslaunch lwr_fri lwr_fri_console.launch 
```
enter
```
control
```
Start your publishing script and choose the corresponding controller.


## To test in simulation
```
roslaunch roslaunch lwr_simple_example sim.launch
rosrun rqt_reconfigure rqt_reconfigure
```

Set parameters for damping_eigVal1 and damping_eigVal2 to maximum value.  


# Move robot to initial position
rostopic pub /lwr/joint_controllers/command_jointos std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:  [0,0.1,0,0,0,0.3,0.1]
- " 


# Visual interface to send commands
rosrun rqt_gui rqt_gui

GO TO > Plugins > Topics > Message Publisher

Visualization of nodes & topics
rosrun rqt_graph rqt_graph

Time frame tress
rosrun rqt_tf_tree rqt_tf_tree 

## Change Tool / Modify files
> Configure > User Group > Log On > Administrator
> Password: KUKA

## Vision system
To use optitrack go here:


# Close to candle position:

[0.0, -0.5, 0.0, -1.9904736280441284, 0.05283664911985397, -1.1450096368789673, 0.0062964847311377525]# wheel-polishing

# Basic launching of the robot
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

# xArm_Amiga_CMU

# Setup
Mainly Gazebo and MoveIt package installation required. Then clone this repository in your `catkin_ws/src` and then `catkin_make`. To install everything related to the xArm, follow the instructions of the original [xarm_ros repository](https://github.com/xArm-Developer/xarm_ros#3-preparations-before-using-this-package) 

## Create Amiga model in Gazebo
1. Move model directory to gazebo directory 
```
cd ~/.gazebo/models
mkdir amiga
cd ~/catkin_ws/src/xarm_ros/xarm_gazebo/models
cp -dir amiga/ ~/.gazebo/models/
```

# Running without custom End-Effector
1. Gazebo 
```
roslaunch xarm_gazebo amiga_xarm6.launch
``` 
Error: No p gain specified for pid.  `Namespace: /xarm/gazebo_ros_control/pid_gains/joint is` expected and can be ignored. </br>

2. MoveIt & xArm_Planner 
```
roslaunch xarm6_moveit_config xarm6_moveit_gazebo_plan.launch
```
Manually move the EE in MoveIt GUI and press `Plan & Execute` to move the robot. Both RViz and Gazebo robot arm should move.

3. Python Script for desired Motion using ROS service to MoveGroup Interface
(if first time, make this python script executable)
```
~/catkin_ws/src/xArm_Amiga_CMU/xarm_planner/scripts
chmod +x xArm_planar_motion.py
python xArm_planar_motion.py
```

![Motion from scripted interface](doc/amiga_script.png)


# Running with custom End-Effector
1. Gazebo 
```
roslaunch xarm_gazebo amiga_xarm6.launch add_gripper:=true
``` 
Error: No p gain specified for pid.  `Namespace: /xarm/gazebo_ros_control/pid_gains/joint is` expected and can be ignored. </br>

2. MoveIt & xArm_Planner 
```
roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch
```
Wait until the terminal prompts `You can start planning now!` message, which means RViz can now take in manual planning requests.
Manually move the EE in MoveIt GUI and press `Plan & Execute` to move the robot. Both RViz and Gazebo robot arm should move. Can manually drag the EE to be in the configuration shown below </br>


![Motion from scripted interface](doc/EE.png)


3. (A) Python Script for desired Motion using ROS service to MoveGroup Interface
```
python xArm_planar_motion.py
```

3. (B) Checking MoveIt planning feasibility with custom end-effector. 
Run this separately from previous MoveIt file. The previous `xarm6_gripper_moveit_gazebo.launch` is planning with a smaller EE and NOT the custom EE. The `demo_customEE_gazebo.launch` file accounts for the right dimension of the gripp.er
```
roslaunch xarm6_moveit_config demo_customEE_gazebo.launch 
```



![Motion from scripted interface](doc/EE2.png)


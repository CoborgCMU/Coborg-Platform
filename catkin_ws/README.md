# Coborg-Platform

COBORG is an exoskeleton platform that will change the world. The vision of this system is to be able to act as a helping hand for it’s operator. We’ve found that people who do overhead manufacturing, for example automation or aircraft assembly, tend to strain their arms after working for long periods of time, and often require a second person to assist them as they work. We want to create a platform that will empower and aid the user as they do their day to day tasks.

## Get Started


### Compile/Build

1. Under /catkin_ws: 

**Note:Darknet and darknet_ros need to be setup seperately since the large weight files and large /data folder in darknet. See vision subsystem readme to setup these two package before catkin_make**

**Note:If there is any error related to missing msg package, catkin_make the specific package seperately before catkin_make the whole system.**

```
Optional: catkin_make <package_name>

catkin_make
```

### Run

The integrated, voice activated mode of the actuated manipulation system incorporates the other subsystems of this product: voice, vision, and main state machine nodes. This mode was run in the SVD encore event to showcase the synergy between the different subsystems and to showcase the use case flow of using the robot arm. To run this mode, the user must run the other subsystem nodes. From a high level, the order of launching subsystems is shown below:

1. RealSense D435i and T265 -> depth camera node
2. Darknet ros 3D model -> vision node
3. Goal post-processing -> goal_getter node
4. Robot model and URDF -> robot model node
5. Voice recognition node -> voice node
6. Main state machine node -> main state machine node
7. MoveIt model interface node -> path planning node
8. Actuated manipulation pose generator node -> path intermidiate node
9. HEBI motor interface node -> motor node

To run the integrated system:
```
source devel/setup.bash
roslaunch main_state_machine main.launch
```

An alternative method is to run an alias command through terminal to run the main.launch script:
```
source devel/setup.bash
coborg
```

To run subsystem seperately, the following commands should be run (user will need multiple terminal tabs and windows to run all these nodes). Make sure to run `source devel/setup.bash` for all new terminal instances:

```
# terminal instance
roslaunch coborg_move demo_hebi_realsense_tf.launch

# terminal instance
roslaunch darknet_ros_3d darknet_ros_3d.launch

# terminal instance
roslaunch goal_getter goal_getter.launch

# terminal instance
roslaunch voice_recog voice.launch

# terminal instance
roslaunch main_state_machine main.launch

# terminal instance
roslaunch coborg_move Integrated_AM_launch.launch

# terminal instance
roslaunch coborg_move KDC_find_hebi_moveit_planner.launch
```
The flow of this mode begins with verifying that the D435i camera is set at the appropriate viewing angle and the camera can see the hands that will be in view at their intended positions. In addition, the robot arm should be in its home/compact position before giving the COBORG voice commands.

The first thing the user will do is position either one or two hands in front and in view of one or both D435i cameras. The user will then recite "Hey COBORG." in the direction of the microphone that is connected to the local computer. An audible confirmation sound will play when the node correctly interprets the initiation sound. The user will then recite "go here" towards the direction of the microphone. The robot will feedback an audible confirmation sound and the vision and actuated manipulation pipeline will begin. After the vision nodes acquire the goal position of the center of the one hand or the average center between the two hands, the actuated manipulation system will acquire that goal pose and initiate its motion. The robot arm will first confirm it is in its home position. Then the robot arm will initiate to its ready position which is partly extended outwards in front of the robot. The arm will then attempt to solve for a position that is some offset distance along the surface normal of the part relative to the global frame of the robot URDF model. Once the COBORG reaches this offset position, it will then initiate naive push and stabilization. By tracking the global goal position, the arm and adjust itself and it is moving towards and providing force on the intended part. Once the user wants the robot arm to retract back, the user will say "Hey COBORG". The robot will feedback a confirmation tone. The user will say "come back". The robot will feedback another audible confirmation tone. The arm will then go back to the ready position and then back to home.

If the user provides an audible "STOP STOP STOP STOP" command to the COBORG, the system will cut power to the motors and the arm will go limb. If the COBORG was stabilizing and holding a part before this emergency stop command, the arm will lose control of the intended part. Make sure to have control of the part to avoid any damage to the user. To start the motors, the user will say the key phrase "Hey COBORG." The robot will feedback an audible confirmation tone. The user will then say "start up" and the system will provide power to the motors. 


## Team
CMU 2020 - MRSD Team C:

Husam Wadi, Yuqing Qin, Gerry D’Ascoli, Feng Xiang, Jonathan Lord-Fonda

BioRobotics Lab at CMU.

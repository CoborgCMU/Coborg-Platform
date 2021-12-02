---
# Jekyll 'Front Matter' goes here. Most are set by default, and should NOT be
# overwritten except in special circumstances. 
# You should set the date the article was last updated like this:
date: 2021-09-14 # YYYY-MM-DD
# This will be displayed at the bottom of the article
# You should set the article's title:
title: Actuated Manipulation System Setup
# The 'title' is automatically displayed at the top of the page
# and used in other parts of the site.
---

## Overview

In this readme, we will discussing the following:

- How to install and configure the COBORG actuated manipulation subsystem
- Run the full pipeline

## Setting up actuated manipulation system

### Requirements

- Ubuntu: 18.04

- Ubuntu: librealsense2 (v2.42.0.0-realsense0.4059)

- Ubuntu: MoveIt 1.0 for ROS Melodic

- Ubuntu: HEBI API for ROS Melodic

- ROS: Melodic (v1.4.1-0bionic.20210304.173654)

- ROS: Realsense2-camera (v2.2.22-1bionic.20210219.07850)

- ROS: Realsense2-description (v2.2.22-1bionic.20210219.071513 500)


### Steps
1. #### Download the repo from the following link:

[CoBorg-Platform GitHub Link](https://github.com/Sunny-Qin-0314/Coborg-Platform/tree/devel_arm64)

Run the following commands to clone the repository and go to the appropriate branch:

```
git clone https://github.com/CoborgCMU/Coborg-Platform.git
cd CoBorg-Platform
git pull
git checkout devel_arm64
cd catkin_ws
```

2. #### Installing realsense-ros and librealsense:

To install realsense-ros, download the repo from:

[realsense-ros GitHub link](https://github.com/IntelRealSense/realsense-ros)

Following Method 2 of the authors' installation process to install the realsense-ros node from the catkin_ws ./src folder.

librealsense is a linux package that needs to be installed separate from the realsense-ros node. To install librealsense on Ubuntu, follow the authors' instructions from the librealsense repo:

[librealsense Linux installation GitHub webpage](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

To run RealSense Viewer, run the following command from terminal:

```realsense-viewer```

To check the RealSense devices connected to the local machine, run the following command from terminal:

```rs-enumerate-devices```

NOTE: As of the creation of this README file, Intel has discontinued sale and support of their Realsense cameras. You ought to remove any Realsense repositories from your sources.list file. Refer to [how to remove malformed line from sources list](https://askubuntu.com/questions/78951/how-do-i-remove-a-malformed-line-from-my-sources-list) for details on how to remove repositories from update list on Ubuntu.

3. #### Build:
Delete any ./build or ./devel folders if present in the catkin_ws folder. Install ros dependencies using the ```rosdep``` command.

```
cd CoBorg-Platform/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make darknet_ros
catkin_make darknet_ros_3d
catkin_make install
source devel/setup.bash
```

NOTE: Will need to catkin_make the darknet_ros node separately first before running catkin_make for the reset of the nodes. This will avoid any unexpected errors from occurring.

# How to Run Full Pipeline

To run the full COBORG pipeline would be to run the system with the following subsystems:
- Main state machine
- Voice subsystem
- Vision subsystem
- Actuated manipulation subsystem

There are two methods to run the full pipeline, and both methods require only one command to terminal:
1. Run a roslaunch command of the main launch file through terminal:
```
roslaunch main_state_machine main.launch
```
2. Run a shortcut command through terminal to launch the full pipeline through xterm
```
coborg
```

Operating the COBORG is performed via four voice commands:
1. "Hey COBORG...Go here"
2. "Hey COBORG...Come back"
3. "STOP STOP STOP STOP"
4. "Hey COBORG...Start up"

Before giving the "Go here" command, make sure to have your hands at the ready position, showing your hands to one or both of the Realsense depth cameras. Once you give the "Go here" command, a feedback signal will be outputted through the speaker to inform the user that the COBORG received the command and is now finding the hands to create a 3D goal pose. Once the goal pose is found, the robot arm will plan and actuate to the goal pose. If the robot arm cannont plan a viable path within a set amoung of time, an error sound will be played and the robot arm will come back to the home position and await the next "Go here" command. 

Should the robot arm actuate to the 3D goal pose, the arm will pause for a set amount of time before playing the audio to start stabilization. From there, a separate controller will actuate the arm into the panel and should provide force to hold the part in place. The user can then move around reasonably and the robot arm will adjust its configuration to maintain that goal pose onto the panel.

To bring the arm back, the user will give the "Come back" voice command. COBORG will output a feedback audio signal to confirm and perform a naive pull to release control of the part. Then the robot arm will plan and actuate to the home position. In the rare chance that the robot arm is unable to actuate to the home position, it may hand at the ready intermediate position. In this case, it would be best to kill the COBORG pipeline and restart it via terminal.

Giving the "STOP STOP STOP STOP" command will trigger a GPIO pin in the computer to cut power off from the HEBI motors. In this case, the arm will go limp and drop down. Make sure to have at least one hand controlling the part when giving the "STOP STOP STOP" command to prevent the part dropping onto the user.

To bring power back to the COBORG arm after initiating "STOP STOP STOP STOP", give the "Start up" command. This will bring power back to the HEBI motors and actuate them to the home position. Should the arm not start at the home position after giving the "Start up" command, it would be best to kill the COBORG pipeline and restart it via terminal.

# Miscellaneous methods to run

Other modes to run the actuated manipulation system are:

1. Manual mode

## Manual Mode

Manual mode involves the user manually quering goal poses to the robot URDF through the ROS visualization tool, RViz. Run the following commands to activate manual mode:

```
roslaunch dof_4_lowerlonger_arm demo.launch
roslaunch coborg_move KDC_find_hebi_moveit_planner.launch
```

Querying a goal pose through RViz is as easy as drag-and-dropping the queried end-effector pose (colored in orange through RViz). You can also set multiple preset positions through Rviz, selecting "Plan" and selecting "Execute" once a successful plan is created. There are also a number of preset arm configurations that you can set as goal poses.


# Miscellaneous Topics

## Converting HRDF to URDF
Note: HRDF file is created using HEBI's 3D CAD tool:

[HEBI 3D configurator website](https://robotbuilder.hebirobotics.com)

In the terminal, navigate to the hebi_description source folder. You will need to run the generate_pipeline.bash tool to convert HRDF to XACRO and SDF.

'''
./scripts/generate_pipeline.bash <HRDF file> <space deliminted desired names of actuator motors>
'''

The SDF file will be saved in the models/ folder. The URDF file will be saved in the urdf/kits/ folder. 
  
NOTE: the bash script assumes that there is a gripper at the end of the linkage arm. Would need to do some manual configuration of the XACRO file to clean up the code and add additional components to the URDF model.

NOTE: when observing the kinematic chain of motors and linkages as described in the xacro file, keep in mind that reporting twist angles are origin-based from the previous motor / joint angle, rather than a global angle origin. 

## Converting XACRO to URDF
Given a configured XACRO .xml robot model file, you can convert that .xml file into a .urdf file using ROS.

1. Make sure to source devel/setup.bash from the ROS catkin workspace
2. Navigate to the folder where the .xml is located
3. Run the following command:
```
rosrun xacro xacro (XACRO-file).xml > (URDF-file).urdf
```

The .urdf formatted file will be saved to the same location as the .xml or to a specified directory.

## Running MoveIt Setup Assistant
To open the MoveIt setup assistant GUI of the existing COBORG, type the following:

```
roslaunch dof_4_lowerlonger_arm setup_assistant.launch
```

This will launch the current MoveIt configuration of the COBORG.

If you are creating another MoveIt node from scratch, you can open a fresh MoveIt setup assistant instance by typing the following:

```
roslaunch moveit_setup_assistant setup_assistant.launch
```

## Running HEBI Scope
1. Download the HEBI Scope application from the HEBI website:

    [HEBI Apps download page](https://www.hebirobotics.com/apps)

2. Open the HEBI Scope folder and go to the bin/ directory

3. Through terminal, type:
    ```
    ./Scope
    ```

4. Make sure the HEBI motors are turned on and connected to the local computer in the IP network.

5. You should see the HEBI motors located at the left of the Scope application. 

## References
- [ROS MoveIt Tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)


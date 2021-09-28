---
# Jekyll 'Front Matter' goes here. Most are set by default, and should NOT be
# overwritten except in special circumstances. 
# You should set the date the article was last updated like this:
date: 2021-09-27 # YYYY-MM-DD
# This will be displayed at the bottom of the article
# You should set the article's title:
title: Main State Machine Setup Readme
# The 'title' is automatically displayed at the top of the page
# and used in other parts of the site.
---

## Introduction

In this readme, we will do the following things for the main state machine:

- Understand what main.py does
- Understand the inputs and outputs for the system
- Understand what rostopics main.py references

---
### Requirements

- Ubuntu: 18.04

- ROS: Melodic

### Overview

main.py is a central hub for all of the inputs and outputs for the Coborg project. 

This node recieves translated commands from the ```/voice commands``` topic and use that to toggle the systems state through ```/state_output``` 

It then listens to ```/state_input``` topic to understand whether or not the commands sent were executed successfully. 
      
main.py is launched with all other nodes executing this command:
```
roslaunch main_state_machine main.launch
```
      
If you want to test the main.py file by itself, run:
```
rosrun main_state_machine main.py
```

### Functions

#### Commands:
0. Pause = Motors stop moving, but maintain with lower torque threshold
1. Stop = Shut off power to motors
2. Target = Move to and hold plate
3. Home = Return to home position

#### Status Outputs:
1. Initializing = Command received, but not executing yet (e.g. detecting hands)
2. Executing = Command being executed (e.g. moving to target)
3. Idle = Command completed/performing holding task, ready for next command (maintaining position in 3d space)
4. Warning = Issue that is continuable.
5. Error = Problem that requires full reset.

#### Status Inputs:
0. Processing = Command recieved and executing
1. Success = Command completed 
2. Warning = Command completed but with issues
3. Error = Command not completed

### ROS Topics

   * voice_commands_sub (`std_msgs::Int32`) commands from `/voice_commands`
   * state_output_pub (`std_msgs::Int32`) state outputs from main.py `/state_output`
   * state_input_sub  (`std_msgs::Int32`) state inputs from other nodes (actuated manipulation) `/state_input`

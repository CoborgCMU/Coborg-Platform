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

This node receives translated commands from the ```/voice_cmd``` topic and use that to output the main node cmd ```/main_cmd```. 

It then listens to integer state inputs from ```/feedback_voice```, ```/feedback_vision```, and ```/feedback_arm``` topic to understand whether or not the commands sent were executed successfully. This feeds into the overall system state ```/feedback_main```. The main node also controls the relay that switches power to the robot arm if "stop stop stop" is heard from the user. 
      
main.py is launched with all other nodes executing this command:
```
roslaunch main_state_machine main.launch
```

The visualization is separated to allow the ability to run on a seperate computer:
```
roslaunch main_state_machine visual.launch
```

If you want to test the main.py file by itself, run:
```
rosrun main_state_machine main.py
```

### Functions

Refer to the state diagram to understand the structure: https://tinyurl.com/coborgstate

#### Main Commands:
      0. RESTART: Powers up robot arm, triggered by "Hey Coborg...Start up"
      1. TARGET: Move to the identified target, triggered by "Hey Coborg ... Go Here"
      2. HOME: Return to the Home (Compact) position, triggered by "Hey Coborg ... Come Back"
      3. READY: Get into the Ready position in front of the user, triggered by "Hey Coborg ... Get Ready"
      4. CELEBRATE: Plays some music, triggered by "Hey Coborg...Successful Fall Validation Demonstration"
      9. STOP: Kills power to robot arm, triggered by "Stop Stop Stop"

These are identical to the voice commands, however we seperated them to allow the main node to control when those commands are executed.

#### Main Feedback:
      0. Idle = Command completed/performing holding task, ready for next command (maintaining position in 3d space)
      1. Initializing = Starting up node
      2. Executing = Command being executed (e.g. moving to target)
      3. Completed = Task finished successfully
      9. Error = Problem that requires full reset.

#### Subsystem Feedback:
      * https://tinyurl.com/coborgstate

### ROS Topics

   * voiceCommand_sub (`std_msgs::Int32`) commands from `/voice_cmd`
   * feedbackVoice_sub (`std_msgs::Int32`) state feedback from voice `/feedback_voice`
   * feedbackVision_sub (`std_msgs::Int32`) state feedback from vision `/feedback_vision`
   * feedbackArm_sub (`std_msgs::Int32`) state feedback from arm `/feedback_arm`
   * mainCommand_pub  (`std_msgs::Int32`) publishes commands to arm `/main__cmd`
   * feedbackMain_pub  (`std_msgs::Int32`) publishes main state`/feedback_main`
   * speaker_pub (`std_msgs::String`) publishes sounds `/speaker`

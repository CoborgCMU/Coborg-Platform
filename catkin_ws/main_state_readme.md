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

## Overview

In this readme, we will do the following things for the main state machine:

- Understand what main.py does
- Understand the inputs and outputs for the system
- Understand what rostopics main.py references

---
### Requirements

- Ubuntu: 18.04

- ROS: Melodic

### main.py Function
      main.py is a central hub for all of the inputs and outputs for the Coborg project. This node recieves translated commands from the ```/voice commands``` topic and use that to toggle the systems state through ```state_output```. It then listens to ```/state_input``` topic to understand whether or not the commands sent were executed successfully. 
      
      main.py is launched with all other nodes executing this command:
      ```roslaunch main_state_machine main.launch```
      
      If you want to test the main.py file by itself, run:
      ```rosrun main_state_machine main.py```
     

### ROS Topics
* #### Published ROS topics:

   * object_detector (`std_msgs::Int8`) Number of detected objects
   * bounding_boxes (`darknet_ros_msgs::BoundingBoxes`) Bounding boxes (class, x, y, w, h) details are shown in `/darknet_ros_msgs`
   * detection_image (`sensor_msgs::Image`) Image with detected bounding boxes

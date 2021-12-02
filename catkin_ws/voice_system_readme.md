---
# Jekyll 'Front Matter' goes here. Most are set by default, and should NOT be
# overwritten except in special circumstances. 
# You should set the date the article was last updated like this:
date: 2021-09-09 # YYYY-MM-DD
# This will be displayed at the bottom of the article
# You should set the article's title:
title: Voice Subsystem README
# The 'title' is automatically displayed at the top of the page
# and used in other parts of the site.
---

## Overview

This readme contains:
- Voice Subsystem description
- Instructions to set up: 
   * pocketsphinx dependencies
   * pocketsphinx-python
   * python audio dependencies
- How to run voice recognition node demo

---
## Subsystem Description

The voice recognition node is the interface between the user and the robot. This node processes raw audio from the system microphone and processes for recognized voice commands. The NLP model active filters background noise to improve voice recognition. The keyword trigger phrase of "Hey Coborg" serves as a barrier to prevent unwanted command recognition.
This voice_recog node also serves as an interface for the ROS framework to the system audio output. Other nodes can publish to a topic, then the audio plays through voice_recog.

### ROS Nodes
* #### voice_recog

### ROS Topics
* #### Subscribed ROS topics:
   * /speaker (`std_msgs::String`) Name of .mp3 file in Coborg-Platform/catkin_ws/src/voice_recog/src/Sounds/
      * EX: msg.data = "jeez.mp3" plays the jeez.mp3 file out of the system speakers.
* #### Published ROS topics:
   * /voice_cmd (`std_msgs::Int32`) Interger code for the recognized voice command
      * 0 - RESTART: Disengage E-stop, triggered by "Hey Coborg ... Start Up"
      * 1 - TARGET: Move to the identified target, triggered by "Hey Coborg ... Go Here"
      * 2 - HOME: Return to the Home (Compact) position, triggered by "Hey Coborg ... Come Back"
      * 3 - READY: Get into the Ready position in front of the user, triggered by "Hey Coborg ... Get Ready"
      * 4 - CELEBRATE: Fun command to play a copyrighted song to celebrate a successful demo, trigged by "Hey Coborg ... Successful Fall Validation Demonstration"
      * 9 - STOP: Soft E-stop command, triggered by "Hey Coborg ... Stop" or "Stop Stop Stop"
   * /feedback_voice (`std_msgs::Int32`) Interger code for the recognized voice command
      * 10 - IDLE: Waiting for a voice command trigger "Hey Coborg"
      * 11 - INIT: The voice node is setting up
      * 12 - PROCESSING: Heard "Hey Coborg", processing following audio for recognized commands
      * 13 - COMPLETED: Command recognition completed (either successfully recognized or no command interpreted)
---

## Setting up pocketsphinx-python and audio input/output libraries

### Requirements

- Ubuntu: 18.04

- ROS: Melodic

- Python: 3.6.9

- Pocketsphinx-python: subdependencies of Sphinxbase and Pocketsphinx

- Python Packages: PyAudio, Pydub, rospkg

### Steps
1. #### Install system dependencies

   ```sudo apt update```
   
   ```sudo apt dist-upgrade```
   
   ```sudo apt install bison```
   
   ```sudo apt install swig```
   
   ```sudo apt install pavucontrol linux-sound-base alsa-base alsa-utils```
   
   ```sudo apt install pulseaudio```
   
   ```sudo apt install libpulse-dev```
   
   ```sudo apt install osspd```
   
   ```sudo apt install ffmpeg```
   
   **(Some dependencies may be installed by default)**

2. #### Download the repo:
   
    ```cd```

    ```git clone --recursive https://github.com/cmusphinx/pocketsphinx-python.git```

    **Note: make sure you have `--recursive` tag when downloading the pocketsphinx-python package**
    **Note 2: the pocketsphinx-python repo can be deleted once the setup steps are completed**
    
3. #### Install Pocketsphinx packages:
    
    ```cd pocketsphinx-python```
    
    ```python3 setup.py install```
    
    ```sudo python3 setup.py install```
    
    
    ```cd deps/sphinxbase```
    
    ```./autogen.sh```
    
    ```./configure```
    
    ```make```
    
    ```sudo make install```
    
    
    ```cd ../pocketsphinx```
    
    ```./autogen.sh```
    
    ```./configure```
    
    ```make```
    
    ```sudo make install```
    

4. #### Set up system variables to load libraries from the required folders:
    
    ```export LD_LIBRARY_PATH=/usr/local/lib```
    
    ```export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig```
    
5. #### Install Python audio & ROS packages:

   * PyAudio:
   
    ```sudo apt-get intall portaudio19-dev python-pyaudio```
    
    ```pip install pyaudio```
   * Pydub:
   
    ```pip install pydub```
   * rospkg:
   
    ```pip install rospkg```
    
6. #### [OPTIONAL] Train the Pocketsphinx model with Seq2Seq G2P Toolkit:

   * Install dependencies:
   
      ```pip3 install tensorflow==1.8```
      
      ```pip3 install tensor2tensor=1.6.6```

   * Clone g2p-seq2seq repo:
      
      ```git clone https://github.com/cmusphinx/g2p-seq2seq.git```
      
   * Follow instructions in repo:
      https://github.com/cmusphinx/g2p-seq2seq
---

### Run the voice demo
`source [Coborg-Platform]/catkin_ws/devel/setup.py`

`roslaunch voice_recog voice.launch`

**Now say "Hey Coborg" and let the fun begin**

---
## References
* PocketSphinx-Python Repo: https://github.com/cmusphinx/pocketsphinx-python
* Building a PocketSphinx language model: https://cmusphinx.github.io/wiki/tutoriallm/
* g2p-seq2seq Repo: https://github.com/cmusphinx/g2p-seq2seq


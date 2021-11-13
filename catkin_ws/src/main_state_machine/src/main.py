#! /usr/bin/env python
# refer to state diagram: https://tinyurl.com/coborgstate

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Char
from std_msgs.msg import Header   
from std_msgs.msg import String
import enum


class voiceCommand(enum.IntEnum):
    RESTART = 0 # Recover from voice trigger E-stop
    TARGET = 1 # Move to and hold plate
    HOME = 2 # Return to home position
    READY = 3 # Go to ready position (currently unused)
    CELEBRATE = 4 # Time to party!
    STOP = 9 # Shut off power to motors (voice trigger E-stop)

class mainState(enum.IntEnum): 
    IDLE = 0 # Waiting for voice input
    INIT = 1 # Starting up node
    PROCESSING = 2 # Executing command
    COMPLETED = 3 # Finished task
    FAULT = 9 # Error State

class voiceState(enum.IntEnum):
    IDLE = 10 # Waiting for audio
    INIT = 11 # Starting up node
    PROCESSING = 12 # Analyzing audio
    COMPLETED = 13 # Completed task
    FAULT = 19 # Error State

class armState(enum.IntEnum):
    IDLE = 20 # Waiting for new command
    INIT = 21 # Starting up node
    PROCESSING = 22 # Executing command 
    COMPLETED = 23 # Finished task
    FAULT = 29 # Error State

class visionState(enum.IntEnum):
    IDLE = 30 #
    INIT = 31 # Starting up node
    PROCESSING = 32 # Executing command (not used)
    COMPLETED = 33 # Found Hand(s)
    FAULT = 39 # Error State

#voice command -> arm movement
def new_command(message):
    command = message.data
    global mainCommand_pub, feedbackMain_pub

    #if normal command
    if command == voiceCommand.TARGET or command == voiceCommand.HOME \
     or command == voiceCommand.READY or command == voiceCommand.CELEBRATE:
        mainCommand_pub.publish(command)
        status = mainState.PROCESSING
        feedbackMain_pub.publish(status)

    #if stop command, toggle relay high (NC so it kills power)
    elif command == voiceCommand.STOP:
        mainCommand_pub.publish(command)
        status = mainState.FAULT
        feedbackMain_pub.publish(status)
        GPIO.output(output_pin, GPIO.HIGH)

    #if restart command, toggle relay low (NC so it powers on)
    elif command == voiceCommand.RESTART:
        mainCommand_pub.publish(command)
        status = mainState.IDLE
        feedbackMain_pub.publish(status)       
        GPIO.output(output_pin, GPIO.LOW)
        
#right now the main state just mirrors the arm state. voice + vision optional logic in the future
def status_update(message): 
    new_status = message.data

    if status != mainState.FAULT: #if not faulted, mirror arm node
        if new_status%10 == 9:
            # voice: 19, arm: 29, vision: 39 error out
            mainCommand_pub.publish(voiceCommand.HOME)  # back to home position and wait for next voice command
            status = mainState.IDLE
            feedbackMain_pub.publish(status)       
            speaker_pub.publish("errorSound.mp3")
            return

        if new_status == armState.IDLE:
            feedbackMain_pub.publish(mainState.IDLE)

        elif new_status == armState.PROCESSING:
            feedbackMain_pub.publish(mainState.PROCESSING)

        elif new_status == armState.COMPLETED:
            feedbackMain_pub.publish(mainState.COMPLETED)


if __name__ == "__main__":  
    status = mainState.INIT
    output_pin = 18
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    rospy.init_node('main_state_machine')
    voiceCommand_sub = rospy.Subscriber('/voice_cmd', Int32, new_command)
    feedbackVoice_sub = rospy.Subscriber('/feedback_voice', Int32, status_update)
    feedbackVision_sub = rospy.Subscriber('/feedback_vision', Int32, status_update) 
    feedbackArm_sub = rospy.Subscriber('/feedback_arm', Int32, status_update)

    mainCommand_pub = rospy.Publisher('/main_cmd', Int32, queue_size=1)
    feedbackMain_pub = rospy.Publisher('/feedback_main', Int32, queue_size=1)
    speaker_pub = rospy.Publisher('/speaker', String, queue_size=1)
    #Usage: speaker_pub.publish("[sound].mp3")

    rospy.spin()

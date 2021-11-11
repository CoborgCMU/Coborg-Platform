#! /usr/bin/env python3

import os
from pocketsphinx import Decoder
import pyaudio
from pydub import AudioSegment
from pydub.playback import play
import math
import enum
import time
import threading

import rospy
import rospkg
from std_msgs.msg import Int32
from std_msgs.msg import String

class Command(enum.IntEnum):
    RESTART = 0
    TARGET = 1
    HOME = 2
    READY = 3
    CELEBRATE = 4
    STOP = 9

class State(enum.IntEnum):
    IDLE = 10
    INIT = 11
    PROCESSING = 12
    COMPLETED = 13

rospack = rospkg.RosPack()
package_dir = rospack.get_path("voice_recog")
voice_dir = package_dir + '/src'
model_dir = package_dir + '/src/model'
sound_dir = package_dir + '/src/Sounds/'

def playSoundCallback(msg):
    mp3 = AudioSegment.from_mp3(sound_dir + msg.data)
    soundthread = threading.Thread(target=play, args=(mp3,))
    soundthread.start()

if __name__ == "__main__":
    # Init decoder
    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(model_dir, 'en-us'))
    config.set_string('-lm', os.path.join(model_dir, 'en-us.lm.bin'))
    config.set_string('-dict', os.path.join(model_dir, 'coborg.dict'))
    config.set_string('-logfn', '/dev/null')
    decoder = Decoder(config)

    # Add searches
    decoder.set_kws('coborg',os.path.join(voice_dir, 'src/keywords.txt')) #Keyword search mode
    decoder.set_lm_file('lm', os.path.join(model_dir, 'en-us.lm.bin')) #Standard english search mode
    decoder.set_search('coborg')

    # Set up pyaudio input stream for microphone listening
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
    stream.start_stream()

    # Initialize ROS
    rospy.init_node('voice_recog')
    voice_commands_pub = rospy.Publisher('/voice_cmd', Int32, queue_size=1)
    voice_state_pub = rospy.Publisher('/feedback_voice', Int32, queue_size=1)

    speaker_sub = rospy.Subscriber('/speaker', String, playSoundCallback)


    voice_state_pub.publish(State.INIT) 

    state = State.INIT
    command_timer = 0
    timer_flag = False
    command = False
    sound = False
    in_speech_bf = False
    decoder.start_utt()
    while not rospy.is_shutdown():
        if state != State.PROCESSING:
            state = State.IDLE
        voice_state_pub.publish(state)
        command = False
        buf = stream.read(1024)
        if buf:
            # Send raw audio to pocketsphinx decoder
            decoder.process_raw(buf, False, False)
            if command_timer > 0 and (time.time() - command_timer) > 5.0:
                timer_flag = True
                print("TIMEOUT")
            # Once speech is detected, keep listening until no more speech is detected before processing command.
            if decoder.get_in_speech() != in_speech_bf or timer_flag:
                in_speech_bf = decoder.get_in_speech()
                # Once speech is completed (decoder.get_in_speech() is set back to false), process phrase
                if not in_speech_bf or timer_flag:
                    decoder.end_utt()

                    # Print hypothesis and switch search to another mode
                    print('Decoder Mode:', decoder.get_search())
                    results = [seg.word for seg in decoder.seg()]
                    results[:] = [word for word in results if word != '<sil>' and word != '</s>' and word != '<s>']
                    

                    # If in "Command Mode" (after 'coborg' is heard), check for command
                    if decoder.get_search() == 'lm':
                        print ('Result:', results)
                        if 'stop' in results:
                            print(repr(Command.STOP))
                            voice_commands_pub.publish(Command.STOP)
                            sound = AudioSegment.from_mp3(sound_dir + 'stopSound.mp3')
                            command = True
                        elif "'go', 'here'" in str(results):
                            print(repr(Command.TARGET))
                            voice_commands_pub.publish(Command.TARGET)
                            sound = AudioSegment.from_mp3(sound_dir + 'commandSound.mp3')
                            command = True
                        elif "'come', 'back'" in str(results):
                            print(repr(Command.HOME))
                            voice_commands_pub.publish(Command.HOME)
                            sound = AudioSegment.from_mp3(sound_dir + 'commandSound.mp3')  
                            command = True
                        elif "'get', 'ready'" in str(results):
                            print(repr(Command.READY))
                            voice_commands_pub.publish(Command.READY)
                            sound = AudioSegment.from_mp3(sound_dir + 'commandSound.mp3')  
                            command = True
                        elif "'start', 'up'" in str(results):
                            print(repr(Command.RESTART))
                            voice_commands_pub.publish(Command.RESTART)
                            sound = AudioSegment.from_mp3(sound_dir + 'startupSound.mp3')
                            command = True
                        elif "'successful', 'fall', 'validation', 'demonstration'" in str(results):
                            print(repr(Command.CELEBRATE))
                            voice_commands_pub.publish(Command.CELEBRATE)
                            sound = AudioSegment.from_mp3(sound_dir + 'celebrategoodtimes.mp3')
                            command = True

                    # Send stop command when "stop stop stop" is outside of "Coborg" trigger
                    if 'stop stop stop' in results:
                            print(repr(Command.STOP))
                            voice_commands_pub.publish(Command.STOP)
                            sound = AudioSegment.from_mp3(sound_dir + 'stopSound.mp3')
                    
                    # Translate to base language model if 'coborg' is heard.
                    # Switch back to trigger model if language model hears a command (plays failure sound if command not valid)
                    if 'hey coborg' in results:
                        sound = AudioSegment.from_mp3(sound_dir + 'triggerSound.mp3')
                        state = State.PROCESSING
                        voice_state_pub.publish(state)
                        decoder.set_search('lm')
                        command_timer = time.time()
                    elif decoder.get_search() == 'lm' and (len(results) > 0 or timer_flag):
                        state = State.COMPLETED
                        voice_state_pub.publish(state)
                        decoder.set_search('coborg')
                        command_timer = 0
                        timer_flag = False
                        if not command:
                            sound = AudioSegment.from_mp3(sound_dir + 'nocommandSound.mp3')
                    
                    # Play response sound (if set)
                    if sound:
                        soundthread = threading.Thread(target=play, args=(sound,))
                        soundthread.start()
                        sound = False
                    decoder.start_utt()
        else:
            break
    decoder.end_utt()
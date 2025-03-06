#!/usr/bin/python3
import rospy
import numpy as np
import time
import speech_recognition as sr
from speech_recognition import AudioData
from std_msgs.msg import String 

rospy.init_node('voice_detection_node', anonymous=False)
pub = rospy.Publisher('voice_txt', String, queue_size=10)

# this is called from the background thread
def callback(recognizer, audio):
    data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
    audio = np.array(data,dtype=np.int16)
    audio_data = AudioData(audio.tobytes(), 16000, 2)
    try:
        spoken_text= r.recognize_google(audio_data, language='en-US')
        print("Google Speech Recognition pensa tu abbia detto: " + spoken_text)
        pub.publish(spoken_text) # Publish audio only if it contains words
    except sr.UnknownValueError:
        print("Google Speech Recognition non riesce a capire da questo file audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

# Initialize a Recognizer
r = sr.Recognizer()
r.dynamic_energy_threshold = False 
r.energy_threshold = 150 # Modify here to set threshold. Reference: https://github.com/Uberi/speech_recognition/blob/1b737c5ceb3da6ad59ac573c1c3afe9da45c23bc/speech_recognition/__init__.py#L332
m = sr.Microphone(device_index=None, sample_rate=16000, chunk_size=1024)

# Calibration within the environment
# we only need to calibrate once, before we start listening
#print("Calibrating...")
#with m as source:
#    r.adjust_for_ambient_noise(source,duration=3)  
#print("Calibration finished")

# start listening in the background
# `stop_listening` is now a function that, when called, stops background listening
print("Recording...")
stop_listening = r.listen_in_background(m, callback)

rospy.spin()

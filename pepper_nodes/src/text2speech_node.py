#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Text2Speech
from optparse import OptionParser
import rospy
import random

class Text2SpeechNode:
    '''
    This class defines a ROS node responsible for speaking and animation of the robot.
    '''

    def __init__(self, ip, port):
        '''
        The costructor creates a session to Pepper and inizializes the services
        '''    
        self.ip = ip     # Pepper ip
        self.port = port # Pepper port
        self.session = Session(ip, port) # Initialize the session
        
        # Requires the following services:
        # ALAnimatedSpeech needed to move Pepper while speaking. 
        # ALSpeakingMovement to personalize Pepper's movements. 
        self.tts = self.session.get_service("ALAnimatedSpeech")
        self.speak_move_service = self.session.get_service("ALSpeakingMovement")

        # Set the configuration to "contextual": Pepper catch key words to play the right animation.
        self.configuration = {"bodyLanguageMode":"contextual"}

        # Customization Pepper's movements: mapping animation tags to key words.
        self.textToTag = {"hello": ["hi", "hello", "greetings", "hey", "goodbye"],
                          "affirmative": ["yes", "yeah", "ok", "okay"],
                          "enthusiastic": ["happy"],
                          "reason": ["think", "repeat","rephrase"],
                          "no": ["no", "negative"],
                          "I": ["me", "i", "pepper", "i'm", "am", "myself", "robot", "bot"]
                         }
        self.speak_move_service.addTagsToWords(self.textToTag) # add mapping to tags 
        
    def say(self, msg):
        '''
        Rececives a text message and call the ALAnimatedSpeech service.
        The robot will play the text of the message and the contextual movement.
        '''    
        msg = msg.speech.lower()
        self.tts.say(msg, self.configuration)
        return "ACK"
    
    def start(self):
        '''
        Starts the node and create the tts service.
        '''    
        rospy.init_node("text2speech_node")
        rospy.Service('tts', Text2Speech, self.say)
        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        ttsnode = Text2SpeechNode(options.ip, int(options.port))
        ttsnode.start()
    except rospy.ROSInterruptException:
        pass

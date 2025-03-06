#!/usr/bin/env python3
from rasa_ros.srv import Dialogue, DialogueResponse
from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse
from std_msgs.msg import String
import requests
import rospy
import json

class Handler:
    '''
    The constructor creates the service proxy object, which is able to make the robot speak
    '''
    def __init__(self):
        # Initializes a ROS service proxy to interact with the Text-to-Speech service
        self.tts = rospy.ServiceProxy("/tts", Text2Speech)

    '''
    This method calls the Text-to-Speech service and sends it the desired text to be played.
    '''
    def call(self, text: str):
        msg = Text2SpeechRequest() # Create a Text2SpeechRequest message
        msg.speech = text  # Set the speech message
        resp = self.tts(text) # Call the service with the provided text

def callback(data):
    '''
    This callback function is called when a message is received on the "voice_txt" topic.
    It sends the input text to Rasa, receives the response and triggers robot animation and speech.
    '''
    # Extract the received text from the message
    input_text = data.data
    
    # Send the request to the Rasa webhook
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'  # Rasa webhook URL
    message = {
        "sender": "bot",  # The sender of the message
        "message": input_text  # The message text to be sent to Rasa
    }
    r = requests.post(get_answer_url, json=message)
    
    # Create an object for the response
    response = DialogueResponse()
    response.answer = ""
    for i in r.json():
        response.answer += i['text'] + ' ' if 'text' in i else ''
   
    print(response.answer)
    
    # Create a Handler object to convert the answer text to speech
    handler = Handler()
    handler.call(response.answer)  # Send the answer text to the Text-to-Speech service
    
def main():
    # Initialize the ROS node
    rospy.init_node('dialogue_server', anonymous=True)
    
    # Subscribe to the "voice_txt" topic and call the callback function when data is received
    rospy.Subscriber("voice_txt", String, callback)
	
    rospy.loginfo("Ready to answer")	

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
import rospy

class PeoplePerceptionNode:
    '''
    This class defines a ROS node responsible for perception people using the robot's sensors.
    '''

    def __init__(self, ip, port):
        self.ip = ip     # Pepper ip
        self.port = port # Pepper port
        self.session = Session(ip, port) # Initialize the session
        
        # Requires the following services:
        # ALMotion to set Pepper into wakeup or rest mode. 
        # ALBasicAwareness to tracking people capturing external stimulis.
        # ALPeoplePerception is used implicitly by ALBasicAwareness and for set the maximum detection range.
        self.motion_proxy = self.session.get_service("ALMotion")
        self.basic_awareness = self.session.get_service("ALBasicAwareness")
        self.people_perception = self.session.get_service("ALPeoplePerception")

        # Set the engagement mode to "FullyEngaged": once Pepper engage a person doesn't search for stimuli anymore.
        self.basic_awareness.setEngagementMode("FullyEngaged")

        # Set tracking stimuli to detect people and sounds: it helps to reduce the complexity and the computational time needed to process the stimuli.
        self.basic_awareness.setStimulusDetectionEnabled("People", True)
        self.basic_awareness.setStimulusDetectionEnabled("Sound", True)

        # Set the maximum detection range to 3 meters.
        self.people_perception.setMaximumDetectionRange(3.0)

    def start_people_perception(self):
        '''Start the people perception and puts the robot to wakeup.'''
        self.motion_proxy.wakeUp()
        self.basic_awareness.setEnabled(True)

    def stop_people_perception(self):
        '''Stop the people perception and puts the robot to rest.'''
        self.basic_awareness.setEnabled(False)
        self.motion_proxy.rest()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207", help="IP address of the robot")
    parser.add_option("--port", dest="port", default=9559, type="int", help="Port number of the robot")
    (options, args) = parser.parse_args()

    node = PeoplePerceptionNode(options.ip, options.port)
    
    rospy.init_node("people_perception_node")
    
    try:
        node.start_people_perception()
        rospy.loginfo("People Perception started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    finally:
        node.stop_people_perception()
        rospy.loginfo("People Perception ended.")
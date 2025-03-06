# -Development-of-a-ROS-Based-architecture-for-the-Pepper-Robot-in-a-Shopping-Mall-Environment
This project presents the development of a ROS-based architecture for reactive communication between the humanoid robot Pepper, acting as a robotic guardian in a shopping mall, and a user.

How to run the project:
1) It is necessary to change the bot directory of the following files:
   progetto/src/rasa_ros/scripts/rasa_action.sh
   progetto/src/rasa_ros/scripts/rasa_server.sh
2) Move to the project folder
3) Run the following commands:
   catkin build
   source devel/setup.bash
   roslaunch rasa_ros dialogue.xml
4) Open a new terminal window and run the following commands:
   source devel/setup.bash
   roslaunch ros_audio_pkg speech_recognition.launch
5) Open a new terminal window and run the following commands:
   source devel/setup.bash
   roslaunch pepper_nodes pepper_bringup.launch nao_ip:=10.0.1.207
   
Note:
If Pepper's ip address is not 10.0.1.207, but is 10.0.1.230:
1) The last command must be:
   roslaunch pepper_nodes pepper_bringup.launch nao_ip:=10.0.1.230
2) It is necessary to change the ip address on line 14 of the file:
   progetto/src/emotion_recognition/emotion_recognition.py


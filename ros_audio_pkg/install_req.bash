#!/bin/bash 
sudo apt install -y libasound-dev ffmpeg portaudio19-dev libportaudio2 libportaudiocpp0
python3 -m pip install pyaudio speechrecognition librosa sounddevice python_speech_features scipy

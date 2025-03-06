#!/usr/bin/python3
import os
import numpy as np
import cv2
import tensorflow as tf
from tensorflow.keras.models import load_model
from emotion_recognition.utils import Session

class EmotionRecognition:
    '''
    This class can be used to perform facial emotion analysis.
    '''

    def __init__(self, ip = "10.0.1.207", port = 9559):        
        """
        Initializes the EmotionRecognition class with the given IP and port.
        This method also sets up the face detector, emotion classifier and the video service for capturing frames.
        """
        self.ip = ip      # Pepper ip
        self.port = port  # Pepper port

        # Initialize face detector
        self.faceProto = "./emotion_recognition/opencv_face_detector.pbtxt"
        self.faceModel = "./emotion_recognition/opencv_face_detector_uint8.pb"
        self.faceNet = cv2.dnn.readNet(self.faceModel, self.faceProto)

        # Initialize emotion classifier
        self.emotionModel = "./emotion_recognition/emotion.hdf5"
        self.emotionNet = load_model(self.emotionModel)
	
	    # List of emotions predicted by the classifier
        self.emotionList = ['surprise', 'fear', 'disgust', 'happiness', 'sadness', 'anger', 'neutral']
        
        self.padding = 0.2 # Padding factor to expand the face detection area when cropping the image
        self.MEANS = np.array([131.0912, 103.8827, 91.4953]) # Mean color values for normalization of images (RGB channels)
        self.INPUT_SIZE = (224, 224) # Size to which images are resized for the classifier

        # Create a session
        self.session = Session(self.ip, self.port)

        # Requires the following service:
        # ALVideoDevice needed to connect to the robot's camera.
        self.video_service = self.session.get_service("ALVideoDevice")

        # Subscribe to the front camera
        self.camera_id = 0  # Front camera
        self.resolution = 2  # Resolution (2 = 640x480)
        self.color_space = 13  # RGB
        self.fps = 20  # Frames per second
        self.video_stream = self.video_service.subscribeCamera("camera", self.camera_id, self.resolution, self.color_space, self.fps)

    def getFaceBox(self, frame, conf_threshold=0.8):
        """
        Detects faces in the given image frame and returns the bounding boxes 
        around the detected faces along with the image with drawn rectangles.
        """    
        frameOpencvDnn = frame.copy()
        frameHeight = frameOpencvDnn.shape[0]
        frameWidth = frameOpencvDnn.shape[1]
        blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)
        self.faceNet.setInput(blob)
        detections = self.faceNet.forward()
        bboxes = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > conf_threshold:
                x1 = int(detections[0, 0, i, 3] * frameWidth)
                y1 = int(detections[0, 0, i, 4] * frameHeight)
                x2 = int(detections[0, 0, i, 5] * frameWidth)
                y2 = int(detections[0, 0, i, 6] * frameHeight)
                bboxes.append([x1, y1, x2, y2])
                cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight / 300)), 8)
        return frameOpencvDnn, bboxes

    def process_emotion(self):
        """
        Captures an image from the front camera, detects faces, processes the detected face and
        predicts the emotion based on the face image using the pre-trained emotion classifier.
        Returns the predicted emotion as a string.
        """       
        # Acquire a frame
        raw_image = self.video_service.getImageRemote(self.video_stream)
        if raw_image is None:
            return "Sorry, I am unable to acquire image from the camera."

        # Decode the image
        width = raw_image[0]
        height = raw_image[1]
        array = np.frombuffer(raw_image[6], dtype=np.uint8)
        frame = array.reshape((height, width, 3))

        # Face detection
        frameFace, bboxes = self.getFaceBox(frame)

        # If there are bounding boxes, select the one with the largest area (closest face)
        if bboxes:
            # Calculate the area of each bounding box (width * height)
            areas = [(bbox[2] - bbox[0]) * (bbox[3] - bbox[1]) for bbox in bboxes]

            # Find the index of the bounding box with the largest area
            max_area_idx = np.argmax(areas)
            
            # Select the bounding box with the largest area
            bbox = bboxes[max_area_idx]

            # Adjust crop
            w = bbox[2] - bbox[0]
            h = bbox[3] - bbox[1]
            padding_px = int(self.padding * max(h, w))
            face = frame[max(0, bbox[1] - padding_px):min(bbox[3] + padding_px, frame.shape[0] - 1),
                         max(0, bbox[0] - padding_px):min(bbox[2] + padding_px, frame.shape[1] - 1)]
            face = face[face.shape[0] // 2 - face.shape[1] // 2: face.shape[0] // 2 + face.shape[1] // 2, :, :]

            # Preprocess image
            resized_face = cv2.resize(face, self.INPUT_SIZE)
            blob = np.array([resized_face.astype(float) - self.MEANS])

            # Predict emotion
            emotionPreds = self.emotionNet.predict(blob)
            emotion = self.emotionList[emotionPreds[0].argmax()]

            return (f"Your current emotion is: {emotion}")
        else:
            return "Sorry, I couldn't detect your face, try asking the question by coming closer to me."


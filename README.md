# Open House Filkom QUADCOPTER 2023: A Face Following Project for DJI Tello Drone

## Introduction
This project aims to create a face following system for DJI Tello drone using Python and OpenCV. The system can track and follow a human face in real time using a simple face detection and recognition algorithm.

## Technologies and Tools
The system uses a pre-trained haarcascade model from [https://www.geeksforgeeks.org/face-detection-using-cascade-classifier-using-opencv-python/] to detect faces in the video stream from the drone camera.
## Features and Functionalities
The system can follow the face with the highest confidence by checking the position of the bounding box around the face. It also maintains a safe distance from the face by calculating the Euclidean distance between the bounding box and a threshold value. The system can be controlled by using a smartphone app that sends commands to the drone via Wi-Fi.

## How to Run and Test
### dependencies
djitellopy == 1.5
numpy == 1.19.3
opencv_python == 4.5.1.48
### how to run
1. install all of the required libraries
2. clone the repository(model already included in this repository)
3. run the main

## References
https://www.youtube.com/watch?v=vDOkUHNdmKs&t=456s&pp=ygUeZGppIHRlbGxvIGRyb25lIGZhY2UgdHJhY2tpbmcg
https://github.com/kinivi/tello-gesture-control

## Conclusion
This project is an example of how Python and OpenCV can be used to create a simple but effective face following system for DJI Tello drone. But sometimes i encounter a few problem with DJI Tello connection


### Documentation
https://github.com/fwzanrico/Open-House-Filkom-QUADCOPTER-2023/assets/65587186/1d43f390-337f-44f9-a79d-8aec0702ea06


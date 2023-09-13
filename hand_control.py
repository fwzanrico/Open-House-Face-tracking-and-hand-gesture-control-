import cv2
import mediapipe as mp
import numpy as np
from djitellopy import tello

drone = tello.Tello()

#drone.connect()



#drone.takeoff(170)
def hand_gesture(hand_point_list):
    thumbs_open = True
    index_open = True
    middle_open = True
    ring_open = True
    pinky_open = True

    # Check if all required landmarks are present
    required_landmarks = [3, 4, 6, 8, 10, 12, 14, 16, 18, 20]
    if all(landmark in hand_point_list for landmark in required_landmarks):
        if hand_point_list[4][0] > hand_point_list[3][0]:
            thumbs_open = False
        if hand_point_list[8][1] > hand_point_list[6][1]:
            index_open = False
        if hand_point_list[12][1] > hand_point_list[10][1]:
            middle_open = False
        if hand_point_list[16][1] > hand_point_list[14][1]:
            ring_open = False
        if hand_point_list[20][1] > hand_point_list[18][1]:
            pinky_open = False
    

    ##tinggal kesepakatan mau gerak gimana
    if (index_open and middle_open) and (not thumbs_open and not ring_open and not pinky_open):
        print("flip forward")
    
    elif thumbs_open and index_open and middle_open and ring_open and pinky_open:
        print("follow my hand")

    elif not(thumbs_open or index_open or middle_open or ring_open or pinky_open):
        print("landing")
    else:
        print("do nothing")
    return [thumbs_open, index_open, middle_open, ring_open, pinky_open]
    


def cam_stream():
    cam = cv2.VideoCapture(0)
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()
    mpDraw = mp.solutions.drawing_utils
    #drone.stream_on()
    while True:
        success , img = cam.read()
        #img = drone.get_frame_read().frame
        img = cv2.flip(img, 1)
        imageRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(imageRGB)
        hand_point = dict()
        if results.multi_hand_landmarks:
            for idx, handLms in enumerate(results.multi_hand_landmarks): # working with each hand
                # Check if this is the right hand
                if results.multi_handedness[idx].classification[0].label == 'Right':
                    for id, lm in enumerate(handLms.landmark):
                        h, w, c = img.shape
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        hand_point[id]=[cx, cy]  # store the coordinates in the dictionary
                    mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

        cv2.imshow("Output", img)
        gesture_result = hand_gesture(hand_point)
        cv2.waitKey(1)
if __name__ == "__main__":
    cam_stream()
"""
https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html
https://gist.github.com/hauptmech/6b8ca2c05a3d935c97b1c75ec9ad85ff
https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/detect_markers.cpp

"""

import numpy as np
import cv2
import time
from math import hypot, atan2, degrees
import random
import socket
import re


import paho.mqtt.client as mqttc
import logging

print('mqtt imported successfully')

logging.basicConfig(level=logging.DEBUG)

#HOST = "192.168.1.254" # raspberrypi
HOST = "raspberrypi"
PORT = 1883
TOPIC = "ESP32/rx/heading"

# ============================================
# UDP function
# can only send strings
# ============================================

def UDP_client(IP, port,message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(bytes(message, "utf-8"),(IP,port))

test = "hello new world"   
udpIP = "192.168.1.115"
udpPORT_test = 5000
udpPORT = 5001
UDP_client(udpIP, udpPORT_test, test)
# ============================================


def on_connect(client, userdata, flags, rc):
    print("Connected with result code {}".format(rc))

def on_disconnect(client, userdata, rc):
    print("Client was disconnected")

def on_message(client, userdata, message):
    print("Message recevied: "+message.payload.decode())
    #print(type(message))

def on_message_red(client, userdata, message):
    print("RED LIGHT" +message.payload.decode("utf-8"))
    redData = message.payload.decode("utf-8")
    if 'on' in redData:
        print("RED LIGHT ON!")
    elif 'off' in redData:
        print("RED LIGHT OFF!")

def on_message_blue(client, userdata, message):
    print("BLUE LIGHT" +message.payload.decode("utf-8"))
    blueData = message.payload.decode("utf-8")
    if 'on' in blueData:
        print("BLUE LIGHT ON!")
    elif 'off' in blueData:
        print("BLUE LIGHT OFF!")


def removeNotNumbers(a):
    #b = re.sub('\D', '', str(a))
    b = re.sub('[^0-9,.]', '', str(a))
    return b


def getIndexOfTuple(iList, index, value):
    for pos,t in enumerate(iList):
        if t[index] == value:
            return pos

    # Matches behavior of list.index
    #raise ValueError("list.index(x): x not in list")


# Declare global var to store Bot location
global botLoc
global bobbinLoc

botLoc = (0,0)
bobbinLoc = (0,0)


# need to fix this once we properaly calibrate the camera
cameraMatrix = np.load('D:/04 ITECH/STUDIO\High LoA Fab/07_COMBINED STUDIO/00_CAMERA/200913_CameraTesting/CameraCalibration/Test01C/camera_data_test/camera_data_testcam_matrix.npy')
cameraCoeff = np.load('D:/04 ITECH/STUDIO\High LoA Fab/07_COMBINED STUDIO/00_CAMERA/200913_CameraTesting/CameraCalibration/Test01C/camera_data_test/camera_data_testdistortion_coef.npy')
print(f'cameraMatrix = {cameraMatrix}')
print(f'camera coefficient = {cameraCoeff}')


# Set the camera to use (might need to switch between 0 and 1 sometimes...)
cap = cv2.VideoCapture(0)
# Load the aruco dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejects = cv2.aruco.detectMarkers(gray,dictionary)


    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(frame,corners,ids,borderColor=(0,255,0))

        #uncomment this once calibration is done
        #vecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, cameraCoeff)
        #print(f' rvec = {vecs[0]}')
        #print(f' tvec = {vecs[1]}')

        locData_list = []
        botID_list = [8, 10] # keep this numbers from low to high
        botLoc_list = []
        d_list  = []
        bobbinLoc_list = []

        for i, idd in enumerate(ids):
            #uncomment this once calibration is done
            #cv2.aruco.drawAxis(frame, cameraMatrix, cameraCoeff, vecs[0][i], vecs[1][i], 0.1)

            # Get the center of each aruco marker
            cX = int((corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4)
            cY = int((corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1])/ 4)
            cv2.circle(frame, (cX, cY), 5, (0,255,0), -1)
            #cv2.putText(frame, "X: " + str(cX) + " Y: " + str(cY), (cX -20, cY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Find the Bots based on markerID
            if idd[0] in botID_list:
                botLoc = (cX, cY)
                botLoc_list.append((idd[0], botLoc))
                cv2.putText(frame, f"BOT {idd[0]}", (botLoc[0] +20, botLoc[1] -20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1 )
            # all other ID's should then be bobbins... store in bobbin list to send to GH
            else:
                bobbinLoc = (cX, cY)
                bobbinLoc_list.append((idd[0], bobbinLoc, 'x'))

        #print(bobbinLoc_list)
        cleanBobbinLoc_list = removeNotNumbers(bobbinLoc_list)
        # Send bobbin locations to GH for vis
        UDP_client(udpIP, udpPORT, str(cleanBobbinLoc_list))


        # Find the distance in camera world to closest bobbin
        for i, idd in enumerate(ids):
            counter = 0
            random.seed(16)
            cX = int((corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4)
            cY = int((corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1])/ 4)    
            for bot in botLoc_list:
                if idd != bot[0] and idd not in botID_list:
                    counter += 15
                    # bot in botLoc_list are tuple (id, location)
                    dist = hypot((bot[1][0] - cX), (bot[1][1] - cY))
                    #########################################
                    # d_list is (bot id, bot loc, dist to closest bob, closest bob id, closest bob loc)
                    d_list.append((bot[0], bot[1], dist, idd[0], (cX, cY)))
                    cv2.putText(frame, f"Dist to bot {bot[0]}: " + str(int(dist)), (cX -20, cY +counter), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    # Draw some lines for fun
                    cv2.line(frame, (bot[1][0],bot[1][1]), (cX,cY),(random.randint(0,255),random.randint(0,255),random.randint(0, 255)), 1)
    

        #print(len(d_list))
        #print(d_list)
    if len(corners) > 0:
        sortedData = sorted(d_list, key=lambda tup: (tup[0],tup[2]))
        #print(sortedData)
        # Sort the list by distance and retrieve the closest bobbin FOR EACH bot
        closestBobList = []
        for i, num in enumerate(botID_list):
            try:
                if num in ids:
                    closestBobIndex = getIndexOfTuple(sortedData, 0, num)
                    closestBobList.append(sortedData[closestBobIndex])
                    #print(closestBobIndex, sortedData[closestBobIndex])
                    cv2.circle(frame, sortedData[closestBobIndex][4], 5, (255,0,255), -1)
                    cv2.line(frame, (sortedData[closestBobIndex][1][0],sortedData[closestBobIndex][1][1]), (sortedData[closestBobIndex][4][0],sortedData[closestBobIndex][4][1]),(255,0,255), 1)
                    print(f'The closest bobbin to BOT {sortedData[closestBobIndex][0]} is BOBBIN: {sortedData[closestBobIndex][3]} at {round(sortedData[closestBobIndex][2], 2)} units')
            
            except:
                print("no bots or bobbins around....") # need to fix this crashing when u cover up bobbins
                pass

        #print(closestBobList)

        """
        tar = closeBobbin[1]
        if tar[0] > botLoc[0] - 40 and tar[0] < botLoc[0] + 40:
            print("arrived X")
            if tar[1] > botLoc[1] - 40 and tar[1] < botLoc[1] + 40:
                print("arrived Y")
        """

        
        # Find the slope from botLoc to tar using botLoc as the "center"
        # botLoc = vec A, tar = vec B
        # https://gamedev.stackexchange.com/questions/69649/using-atan2-to-calculate-angle-between-two-vectors
        # we solving for the green angle in the above link
        for i in range(0, len(closestBobList)):
            tar = closestBobList[i][4]
            dx = closestBobList[i][1][0] - closestBobList[i][4][0] # vecA.x - vecB.x
            dy = closestBobList[i][1][1] -closestBobList[i][4][1] # vecA.y - vecB.y

            rads = atan2(dy, dx) 
            degs = degrees(rads)
            if degs < 0:
                degs += 360
            elif degs > 360:
                degs -= 360
            cv2.putText(frame, "tar heading: " + str(int(round(degs, 1))), (closestBobList[i][1][0] -20, closestBobList[i][1][1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            #print(degs)


    # =======================================
    # mqtt
    client = mqttc.Client() # pretty sure this only works with the pubsublibrary on MQTTv3
    client.enable_logger(logging.getLogger())
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect(HOST, PORT)
    client.loop_start() # now the client keeps checking on subbed msgs
    logging.info("Loop has started...")

    #client.subscribe("ESP32/rx") # #(hashtag) subscribes to all topics
    client.subscribe("ESP32/#")
    #client.message_callback_add("ESP32/pub/red", on_message_red)
    #client.message_callback_add("ESP32/pub/blue", on_message_blue)

    client.publish(TOPIC,degs, qos=2, retain=False)

    # =========================================


    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.imshow('gray', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()




"""
code for testing corners i.e. what is what
            print(idd)
            print('--------------')
            print(corners[i][0])
            print('--------------')
            print(corners[i][0][0])
            print(corners[i][0][1])
            print(corners[i][0][2])
            print(corners[i][0][3])
            print('--------------')
            print(corners[i][0][0][0])
            print(corners[i][0][0][1])
            print('--------------')
            # corners[i] = the corners for ith id.
            # corners[i][0] = gets rid of the outermost nested bracket [[[]]] becomes [[]]
            # corners[i][0][0123] = gets the XY of each corner []
            # corners[i][0][0123][01] = gets the X or Y of each corner
            time.sleep(2)
"""



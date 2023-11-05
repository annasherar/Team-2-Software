#!/usr/bin/env python

import rospy
import cv2
#from std_msgs.msg import Header
#from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import sys


def aruco_node_py():
    #Initialize Node
    rospy.init_node('aruco_node', anonymous = True)

    #Create an image publisher object for the detected aruco markers image
    #image_pub = rospy.Publisher('webcam/image', Image, queue_size = 10)

    #Use open cv to get camera footage
    webCam = cv2.VideoCapture(0)

    #Set the publishing rate (here its 10hz)
    rate = rospy.Rate(60)

    #Initialize Aruco marker detection variables
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    detectorParams = cv2.aruco.DetectorParameters_create()

    while not rospy.is_shutdown():
        ret, webCamFrame = webCam.read() #returns a boolean and image

        #If webCam.read returned true (it captured a camera frame), then show
        if(ret):

            #Detect ArucoMarkers
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(webCamFrame, arucoDict, parameters = detectorParams)
          
        #If markers were detected, draw bounding box
        if (markerIds is not None):
            cv2.aruco.drawDetectedMarkers(webCamFrame, markerCorners, markerIds)

        #display webcam feed with detect aruco marker bounding boxes
        cv2.imshow("Web Cam", webCamFrame)
        cv2.waitKey(1)

        rate.sleep()



if __name__ == '__main__':
    try:
        aruco_node_py()
    except rospy.ROSInterruptException:
        pass
        
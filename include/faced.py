#!/usr/bin/env python

import roslib; roslib.load_manifest('face')

import numpy as np
import rospy
import cv2
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from face.msg import vector
from rospkg import RosPack

faceCascade = None 
bridge = None 

def talker(u, v):
    pub = rospy.Publisher('sendVector', vector, queue_size = 10)
    rospy.init_node('face')
    r = rospy.Rate(10) #10hz
    msg = vector()
    msg.x = u
    msg.y = v
    pub.publish(msg)
    r.sleep()
    
def callback(data):

    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(50, 50),
        maxSize=(150, 150)
    )

    if len(faces) == 1:
        print "Found {0} faces!".format(len(faces))
    else:
        talker(0, 0)

    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(image, (x+w/2, y+h/2), 3, (0, 0, 255), 5)
        cv2.circle(image, (x+w/2, image.shape[0]/2), 3, (0, 0, 255), 2)
        diff = x+w/2 - image.shape[1]/2
        talker(int(x+w/2), int(y+h/2))

    cv2.circle(image, (image.shape[1]/2, image.shape[0]/2), 2, (255, 0, 0), 5)

    cv2.imshow('Detector de Rostros', image)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('face')
    print('Cargando el Detector de Rostros')
    rp = RosPack() 
    faceCascade = cv2.CascadeClassifier('/home/robert/ros_wokspace/src/face/include/haarcascade_frontalface_default.xml')
    bridge = CvBridge()
    sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback, queue_size=10)
    print('Video AR-Drone Cargado')
    rospy.spin()


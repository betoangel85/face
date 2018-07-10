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
    r = rospy.Rate(100)
    msg = vector()
    msg.x = u
    msg.y = v
    pub.publish(msg)
    r.sleep()

def	callbackBW(ros_image):
    try:
        frame =	bridge.imgmsg_to_cv2(ros_image, 'bgr8')
    except CvBridgeError, e:
        print e

    frame =	np.array(frame,	dtype=np.uint8)
    display_image =	process_image(frame)
    cv2.imshow('Imagen Blanco y Negro', display_image)

    cc = 'x'
    keystroke = cv2.waitKey(5)
    if 32 <= keystroke	and	keystroke < 128:
        cc = chr(keystroke).lower()
    if cc == 'q':
        rospy.signal_shutdown('Presiona q para salir.')

def	process_image(frame):
    grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    grey = cv2.blur(grey, (7, 7))
    edges =	cv2.Canny(grey,	15.0, 30.0)
    return edges

    
def callback(data):
    image = bridge.imgmsg_to_cv2(data,'bgr8') # desired_encoding="passthrough")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(60, 60),
        maxSize=(120, 120)
    )

    if len(faces) == 1:
        print "{0} rostros encontrados".format(len(faces))
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
    
    cc = 'x'
    keystroke = cv2.waitKey(5)
    if 32 <= keystroke and keystroke < 128:
        cc = chr(keystroke).lower()
    if cc == 'q':
        rospy.signal_shutdown('Presiona q para salir.')


if __name__ == "__main__":
    rospy.init_node('face')
    print('Cargando el Detector de Rostros')
    rp = RosPack() 
    file = rospy.get_param('~detector_file')
    faceCascade = cv2.CascadeClassifier(file)
    bridge = CvBridge()
    camera = rospy.get_param('~image_topic')
    print(file)
    print(camera)
    sub = rospy.Subscriber(camera, Image, callback, queue_size=10)
    subBW = rospy.Subscriber(camera, Image, callbackBW, queue_size=10)
    print('Video AR-Drone Cargado')
    rospy.spin()


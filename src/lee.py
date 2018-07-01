#!/usr/bin/env python

import rospy
from face.msg import vector

def callback(data):
    rospy.loginfo("x = %d y = %d" % (data.x, data.y))

def listener():
    rospy.init_node("lee", anonymous=True)
    rospy.Subscriber("escribe",vector, callback)
    rospy.loginfo("Nodo Creado")
    rospy.spin()

if __name__ == '__main__':
    listener()
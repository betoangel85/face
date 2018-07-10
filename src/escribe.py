#!/usr/bin/env python

import rospy
from face.msg import vector

def talker():
    pub = rospy.Publisher('lee', vector)
    rospy.init_node('escribe')
    r = rospy.Rate(10) #10hz
    msg = vector()
    msg.x = 100
    msg.y = 4

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

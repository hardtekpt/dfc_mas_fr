#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys


def Node(id:int):
    pub_vel = rospy.Publisher('node'+str(id)+'/vel_cmd', Twist, queue_size=10)
    rospy.init_node('node'+str(id))

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        v = Twist()
        v.linear.x = 1
        v.linear.y = 1
        v.linear.z = 1
        pub_vel.publish(v)
        rate.sleep()

if __name__ == '__main__':
    try:
        Node(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass

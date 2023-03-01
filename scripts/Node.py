#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import numpy as np

from dfc_mas_fr.Algorithm import Algorithm
from dfc_mas_fr.GradientMap import GradientMap

def get_map():

    map_dimensions = np.asarray([4, 7])
    map = GradientMap(dimensions=map_dimensions)

    hiperboloid_center = np.asarray([2, 3.5])
    hiperboloid_params = np.asarray([5000, 3, 3, 15])
    map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

    return map

def Node(id:int):

    pub_vel = rospy.Publisher('node'+str(id)+'/cmd_vel', Twist, queue_size=10)
    rospy.init_node('node'+str(id))

    SR = 15
    map = get_map()
    cw = np.asarray([0.07, 0.01, 0.6, 0.08, 1, 1])

    al = Algorithm(SR, map, cw)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # Get current positions
        # Get current velocities
        # Get distribution

        #vel = al.update_movement(id)

        v = Twist()
        v.linear.x = 0.1
        v.linear.y = 0
        v.linear.z = 0
        pub_vel.publish(v)
        rate.sleep()

if __name__ == '__main__':
    try:
        Node(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass

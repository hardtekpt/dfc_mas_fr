#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from dfc_mas_fr.Algorithm import Algorithm
from dfc_mas_fr.NodeHelper import NodeHelper
from dfc_mas_fr.srv import Commander
from geometry_msgs.msg import Twist
from dfc_mas_fr.msg import MapUpdate


def Node(id:int):

    SR = rospy.get_param("agent_sensing_radius")
    n_agents = rospy.get_param("number_of_agents")
    nh = NodeHelper(n_agents, SR, id)
    
    pub_vel = rospy.Publisher('node'+str(id)+'/cmd_vel', Twist, queue_size=10)
    rospy.init_node('node'+str(id))
    rate = rospy.Rate(30)
    rospy.Service('node'+str(id)+'/commander', Commander, nh.handle_commander)
    
    map = nh.get_map()
    rospy.Subscriber('publisher/map_update', MapUpdate, map.map_update)

    cw = np.asarray(rospy.get_param("control_weights"))

    al = Algorithm(id, SR, map, cw)

    while not nh.run:
        rate.sleep()

    print("Node" + str(id) + " starting")
    
    while nh.run:

        vel = np.ndarray((3,))
        curr_pos_true = nh.get_curr_pos()
        curr_vel_true = nh.get_curr_vel()
        curr_dist = nh.get_dist()

        curr_pos = nh.corrupt_pos(curr_pos_true, 0.05)
        curr_vel = nh.corrupt_vel(curr_vel_true, 0.05)
        
        vel = al.update_movement(curr_pos[:,0:2], curr_vel[:,0:2], curr_dist)

        v = Twist()
        v.linear.x = vel[0]
        v.linear.y = vel[1]
        v.linear.z = 0
        pub_vel.publish(v)
        rate.sleep()

    print("Node" + str(id) + " stopping")    

if __name__ == '__main__':
    try:
        Node(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass

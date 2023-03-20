#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from dfc_mas_fr.Algorithm import Algorithm
from dfc_mas_fr.NodeHelper import NodeHelper
from dfc_mas_fr.srv import Commander
from geometry_msgs.msg import Twist, PoseStamped
from dfc_mas_fr.msg import Map
from dfc_mas_fr.GradientMap import GradientMap

def Node(id:int):

    SR = rospy.get_param("agent_sensing_radius")
    n_agents = rospy.get_param("number_of_agents")
    max_speed = rospy.get_param("agent_max_vel")
    freq = rospy.get_param("compute_frequency")
    cf_radius = rospy.get_param("cf_radius")
    noise_std = rospy.get_param("noise_std")
    cw = np.asarray(rospy.get_param("control_weights"))

    collision_params = {'max_speed': max_speed, 'time_step_size': 1 / freq, 'cf_radius': cf_radius, 'noise_std': noise_std}
    pub_vel = rospy.Publisher('node'+str(id)+'/cmd_vel', Twist, queue_size=10)

    rospy.init_node('node'+str(id))
    rate = rospy.Rate(freq)
    
    nh = NodeHelper(n_agents, SR, id)
    rospy.Service('node'+str(id)+'/commander', Commander, nh.handle_commander)
    map = GradientMap()
    rospy.Subscriber('publisher/map', Map, map.map_update)

    al = Algorithm(id, SR, map, cw, collision_params)

    while (not nh.run) or (rospy.Time.secs == 0):
        rate.sleep()

    print("Node" + str(id) + " starting")
    
    while nh.run:

        vel = np.zeros((3,))
        curr_pos_true = nh.get_curr_pos()
        curr_vel_true = nh.get_curr_vel()
        curr_dist = nh.get_dist()

        curr_pos = nh.corrupt_pos(curr_pos_true, noise_std)
        curr_vel = nh.corrupt_vel(curr_vel_true, noise_std)

        nh.publish_estimated_pos(curr_pos[id-1])
        
        vel = al.update_movement(curr_pos[:,0:2], curr_vel[:,0:2], curr_dist)
        vel = al.check_for_obstacle_collisions(vel, curr_pos[:,0:2])
        vel = al.check_for_boundaries(vel, curr_pos[:,0:2])
        vel = al.check_for_agent_collisions(vel, curr_pos[:,0:2], curr_dist)

        if np.linalg.norm(vel) > max_speed:
            vel = al.normalize(vel) * max_speed

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

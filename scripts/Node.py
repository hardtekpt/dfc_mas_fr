#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from dfc_mas_fr.Algorithm import Algorithm
from dfc_mas_fr.NodeHelper import NodeHelper
from dfc_mas_fr.srv import Commander
from geometry_msgs.msg import Twist
from dfc_mas_fr.msg import Map
from dfc_mas_fr.GradientMap import GradientMap
from dfc_mas_fr.EKF import EKF

def Node(id:int):

    SR = rospy.get_param("agent_sensing_radius")
    n_agents = rospy.get_param("number_of_agents")
    max_speed = rospy.get_param("agent_max_vel")
    freq = rospy.get_param("compute_frequency")
    cf_radius = rospy.get_param("cf_radius")
    noise_std = rospy.get_param("noise_std")
    cw = np.asarray(rospy.get_param("control_weights"))
    pos_and_vel_data_received = False

    collision_params = {'max_speed': max_speed, 'time_step_size': 1 / freq, 'cf_radius': cf_radius, 'noise_std': noise_std}
    pub_vel = rospy.Publisher('node'+str(id)+'/cmd_vel', Twist, queue_size=10)

    rospy.init_node('node'+str(id))
    rate = rospy.Rate(freq)
    
    nh = NodeHelper(n_agents, SR, id)
    rospy.Service('node'+str(id)+'/commander', Commander, nh.handle_commander)
    map = GradientMap()
    rospy.Subscriber('publisher/map', Map, map.map_update)

    al = Algorithm(id, SR, map, cw, collision_params)

    while (not nh.run) or (rospy.Time.secs == 0) or (not pos_and_vel_data_received):
        pos_and_vel_data_received = True
        for i in range(n_agents):
            if (nh.get_curr_pos()[i].position.x == None) or (nh.get_curr_vel()[i].linear.x == None):
                pos_and_vel_data_received = False
        rate.sleep()

    # Kalman filter
    f = EKF(dim_x=4, dim_z=2)

    # state vector [x, y, Vx, Vy]
    f.x_hat = np.array([[nh.get_curr_pos()[id-1].position.x], [nh.get_curr_pos()[id-1].position.y], [0], [0]])

    f.H = np.array([[1, 0, 0, 0],[0, 1, 0, 0]])
    f.Q *= 0.1
    f.R *= noise_std * 2
    f.B[2,0] = 1
    f.B[3,1] = 1
    f.F[0,2] = 0.05
    f.F[1,3] = 0.05
        
    print("Node" + str(id) + " starting")

    p = [nh.get_curr_pos()[id-1].position.x,nh.get_curr_pos()[id-1].position.y,nh.get_curr_pos()[id-1].position.z]
    vel = np.zeros((2,1))
    
    while nh.run:

        
        curr_pos_true = nh.get_curr_pos()
        curr_vel_true = nh.get_curr_vel()
        curr_dist = nh.get_dist()

        curr_pos = nh.corrupt_pos(curr_pos_true, noise_std)
        curr_vel = nh.corrupt_vel(curr_vel_true, noise_std)

        f.predict_step(vel)
        f.update_step(curr_pos[id-1,0:2])        

        p = [f.x_hat[0,0], f.x_hat[1,0], curr_pos[id-1,2]]

        #p = curr_pos[id-1,:]


        vel = al.update_movement(curr_pos[:,0:2], curr_vel[:,0:2], curr_dist)
        vel = al.check_for_obstacle_collisions(vel, p[0:2])
        vel = al.check_for_boundaries(vel, p[0:2])
        vel = al.check_for_agent_collisions(vel, curr_pos[:,0:2], curr_dist)

        

        if np.linalg.norm(vel) > max_speed:
            vel = al.normalize(vel) * max_speed

        
        nh.publish_estimated_pos(p)

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

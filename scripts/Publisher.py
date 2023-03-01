#!/usr/bin/env python

import rospy
import sys
from os.path import dirname, abspath
sys.path.insert(0, dirname(dirname(dirname(abspath(__file__))))+"/crazyswarm/scripts")
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Pose
import numpy as np

from dfc_mas_fr.srv import GetUtilityValue, GetUtilityValueResponse
from dfc_mas_fr.GradientMap import GradientMap

from pycrazyswarm import *

def position_publisher(pub:rospy.Publisher, swarm):

    curr_pos = np.ndarray((len(pub),),Pose)

    for i in range(len(pub)):
        pose = PoseStamped()
        pos = Pose()
        pos.position.x = swarm.allcfs.crazyflies[i].position()[0]
        pos.position.y = swarm.allcfs.crazyflies[i].position()[1]
        pos.position.z = swarm.allcfs.crazyflies[i].position()[2]
        pos.orientation.w = 0
        pos.orientation.x = 0
        pos.orientation.y = 0
        pos.orientation.z = 0
        pose.pose = pos
        pose.header.stamp = swarm.timeHelper.time()
        pose.header.frame_id = "map"
        pub[i].publish(pose)
        curr_pos[i] = pos
    return curr_pos

def velocity_publisher(pub, th, curr_pos, prev_pos, step):

    for i in range(len(pub)):
        vel_stamped = TwistStamped()
        v = Twist()
        v.linear.x = (curr_pos[i].position.x-prev_pos[i].position.x)/step
        v.linear.y = (curr_pos[i].position.y-prev_pos[i].position.y)/step
        v.linear.z = (curr_pos[i].position.z-prev_pos[i].position.z)/step
        v.angular.x = 0
        v.angular.y = 0
        v.angular.z = 0
        vel_stamped.header.stamp = th.time()
        vel_stamped.header.frame_id = "map"
        vel_stamped.twist = v
        pub[i].publish(vel_stamped)

def vel_callback(v:Twist, args):

    cf = args[0]
    cf.cmdVelocityWorld([v.linear.x, v.linear.y, v.linear.z],0)

def algorithm(swarm, th):

    pub_pos = np.ndarray((len(swarm.allcfs.crazyflies),),rospy.Publisher)
    pub_vel = np.ndarray((len(swarm.allcfs.crazyflies),),rospy.Publisher)
    cmd_vel_sub = np.ndarray((len(swarm.allcfs.crazyflies),),rospy.Subscriber)
    curr_pos = np.ndarray((len(pub_pos),),Pose)
    prev_pos = np.ndarray((len(pub_pos),),Pose)
    rate = 30
    first_iteration = True
    duration = 15
    takeoff_and_land_duration = 3

    for i in range(len(pub_pos)):
        pub_pos[i] = rospy.Publisher('node'+str(i+1)+'/pose', PoseStamped, queue_size=10)
        pub_vel[i] = rospy.Publisher('node'+str(i+1)+'/twist', TwistStamped, queue_size=10)

    # takeoff
    swarm.allcfs.takeoff(targetHeight=1, duration=takeoff_and_land_duration)
    th.sleep(3)

    # run algorithm
    for cf in swarm.allcfs.crazyflies:
        cmd_vel_sub[cf.id-1] = rospy.Subscriber('node'+str(cf.id)+'/cmd_vel', Twist, vel_callback, (cf,))

    init_time = th.time()

    while (th.time()-init_time)<duration:
        # publish agent positions
        curr_pos = position_publisher(pub_pos, swarm)
        if first_iteration:
            prev_pos = curr_pos
        # publish agent velocities
        velocity_publisher(pub_vel, th, curr_pos, prev_pos, 1/rate)
        prev_pos = curr_pos
        th.sleepForRate(rate)

    for cf in swarm.allcfs.crazyflies:
        cmd_vel_sub[cf.id-1].unregister()
    th.sleep(1)

    # land
    swarm.allcfs.land(targetHeight=0.04, duration=takeoff_and_land_duration)
    th.sleep(3)

if __name__ == '__main__':

    crazyflies_yaml = dirname(dirname(abspath(__file__)))+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    th = swarm.timeHelper

    rospy.init_node('publisher')
    algorithm(swarm, th)
    rospy.spin()

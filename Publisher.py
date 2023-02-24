#!/usr/bin/env python3

import rospy
import sys
from os.path import dirname, abspath
sys.path.insert(0, dirname(dirname(abspath(__file__)))+"/crazyswarm/scripts")
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np
import threading

from pycrazyswarm import *

def Publisher(swarm):

    rate = rospy.Rate(50) # 10hz
    pub_pos = np.ndarray((len(swarm.allcfs.crazyflies),),rospy.Publisher)

    for i in range(len(pub_pos)):
        pub_pos[i] = rospy.Publisher('node'+str(i+1)+'/pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        for i in range(len(pub_pos)):
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
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pub_pos[i].publish(pose)
            rate.sleep()

def vel_callback(v:Twist):
    #print(v.linear)
    pass

def algorithm(swarm):
    swarm.allcfs.takeoff(targetHeight=1, duration=1.0+2)
    swarm.timeHelper.sleep(2 + 2)
    swarm.allcfs.land(targetHeight=0.04, duration=3)
    swarm.timeHelper.sleep(3)

if __name__ == '__main__':
    crazyflies_yaml = dirname(abspath(__file__))+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)

    rospy.init_node('publisher')
    
    rospy.Subscriber('node1/vel_cmd', Twist, vel_callback)
    x = threading.Thread(target=Publisher, args=(swarm,))
    x.start()
    algorithm(swarm)
    rospy.spin()

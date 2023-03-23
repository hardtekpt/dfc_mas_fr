#!/usr/bin/env python3

import rospy
import message_filters
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Pose
from dfc_mas_fr.srv import CommanderResponse

class NodeHelper:

    def __init__(self, n_agents, SR, id):

        self.n_agents = n_agents
        self.SR = SR
        self.id = id
        self.run = False

        self.pos_subs = np.ndarray((n_agents,),rospy.Subscriber)
        self.vel_subs = np.ndarray((n_agents,),rospy.Subscriber)

        initial_pos_val = Pose()
        initial_pos_val.position.x = None
        initial_pos_val.position.y = None
        initial_pos_val.position.z = None
        self.curr_pos = np.full((n_agents,), initial_pos_val, dtype=Pose)

        initial_vel_val = Twist()
        initial_vel_val.linear.x = None
        initial_vel_val.linear.y = None
        initial_vel_val.linear.z = None
        self.curr_vel = np.full((n_agents,), initial_vel_val, dtype=Twist)
        
        self.dist = np.zeros((n_agents,))

        for i in range(n_agents):
            self.pos_subs[i] = rospy.Subscriber('node'+str(i+1)+'/pose', PoseStamped, self.save_pos, (i,))
            self.vel_subs[i] = rospy.Subscriber('node'+str(i+1)+'/twist', TwistStamped, self.save_vel, (i,))

        self.pub_pos_estimate = rospy.Publisher('node'+str(id)+'/pose_estimate', PoseStamped, queue_size=10)


    def save_pos(self, p:PoseStamped, args):
        
        i = args[0]
        self.curr_pos[i] = p.pose

    def save_vel(self, v:TwistStamped, args):

        i = args[0]
        self.curr_vel[i] = v.twist

    def get_curr_pos(self):

        return self.curr_pos

    def get_curr_vel(self):

        return self.curr_vel

    def get_dist(self):

        for i in range(self.n_agents):
            my_pose = Pose()
            my_pose = self.curr_pos[self.id - 1]
            possible_neighbour_pose = Pose()
            possible_neighbour_pose = self.curr_pos[i]

            my_pos = np.asarray([my_pose.position.x, my_pose.position.y, my_pose.position.z])
            possible_neighbour_pos = np.asarray([possible_neighbour_pose.position.x, possible_neighbour_pose.position.y, possible_neighbour_pose.position.z])
            d = np.linalg.norm(my_pos - possible_neighbour_pos)
            if d <= self.SR:
                self.dist[i] = 1

        return self.dist

    def corrupt_pos(self, data, std_d):

        corrupted_data = np.ndarray((len(data),3))
        noise = np.random.normal(0,std_d,3)

        for i in range(len(noise)):
            if noise[i] > 3 * std_d:
                noise[i] = 3 * std_d
            if noise[i] < -3 * std_d:
                noise[i] = -3 * std_d
        
        for i in range(len(data)):
            pose = Pose()
            pose = data[i]
            corrupted_data[i,0] = pose.position.x + noise[0]
            corrupted_data[i,1] = pose.position.y + noise[1]
            corrupted_data[i,2] = pose.position.z + noise[2]
        return corrupted_data

    def corrupt_vel(self, data, std_d):

        corrupted_data = np.ndarray((len(data),3))
        noise = np.random.normal(0,std_d,3)

        for i in range(len(noise)):
            if noise[i] > 3 * std_d:
                noise[i] = 3 * std_d
            if noise[i] < -3 * std_d:
                noise[i] = -3 * std_d

        for i in range(len(data)):
            twist = Twist()
            twist = data[i]
            corrupted_data[i,0] = twist.linear.x + noise[0]
            corrupted_data[i,1] = twist.linear.y + noise[1]
            corrupted_data[i,2] = twist.linear.z + noise[2]
        return corrupted_data

    def handle_commander(self, req):
        
        if req.start:
            self.run = True
        if req.stop:
            self.run = False
        return CommanderResponse(True)
    
    def publish_estimated_pos(self, p):

        pose_estimate = PoseStamped()
        pose_estimate.header.stamp = rospy.Time.now()
        pose_estimate.pose.position.x = p[0]
        pose_estimate.pose.position.y = p[1]
        pose_estimate.pose.position.z = p[2]
        self.pub_pos_estimate.publish(pose_estimate)
    
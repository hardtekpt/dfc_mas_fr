#!/usr/bin/env python3

import rospy
import numpy as np
from enum import Enum
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Pose
from rosgraph_msgs.msg import Clock
from dfc_mas_fr.srv import Commander
from dfc_mas_fr.MapUpdateHelper import MapUpdateHelper

CH = Enum('Commander_Handler', ['TAKEOFF', 'TAKINGOFF', 'FLY', 'FLYING', 'LAND', 'LANDING'])
class PublisherHelper:

    def __init__(self, swarm):

        self.swarm  = swarm
        self.th = swarm.timeHelper
        self.number_of_agents = rospy.get_param("number_of_agents")
        self.run_duration = rospy.get_param("run_duration")
        self.takeoff_and_land_duration = rospy.get_param("takeoff_and_land_duration")
        self.flying_height = rospy.get_param("flying_height")

        self.pub_pos = np.ndarray((len(self.swarm.allcfs.crazyflies),),rospy.Publisher)
        self.pub_vel = np.ndarray((len(self.swarm.allcfs.crazyflies),),rospy.Publisher)
        self.cmd_vel_sub = np.ndarray((len(self.swarm.allcfs.crazyflies),),rospy.Subscriber)
        
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

        self.rate = rospy.get_param("publish_rate")
        self.map_handler = MapUpdateHelper()
        self.commander_handler = CH.TAKEOFF

        for i in range(len(self.swarm.allcfs.crazyflies)):
            cf = self.swarm.allcfs.crazyflies[i]
            self.pub_pos[i] = rospy.Publisher('node'+str(cf.id)+'/pose', PoseStamped, queue_size=10)
            self.pub_vel[i] = rospy.Publisher('node'+str(cf.id)+'/twist', TwistStamped, queue_size=10)
            self.cmd_vel_sub[i] = rospy.Subscriber('node'+str(cf.id)+'/cmd_vel', Twist, self.vel_callback, (cf,))

    def run_algorithm(self):

        init_time = self.th.time()
        ii = 0
        while (not self.th.isShutdown()):

            # Publish agent positions and velocities
            curr_pos = self.position_publisher(self.pub_pos)
            self.velocity_publisher(self.pub_vel)

            # Publish map
            if ii == self.rate:
                ii = 0
                self.map_handler.map_publisher(curr_pos)

            # Change from takeoff mode to fly mode after the takeoff duration
            if (self.commander_handler == CH.TAKINGOFF) and ((self.th.time()-init_time)>=self.takeoff_and_land_duration+1):
                self.commander_handler = CH.FLY

            # Change from fly to land mode after the run duration
            if (self.commander_handler == CH.FLYING) and (self.th.time()-init_time)>=self.run_duration+self.takeoff_and_land_duration:
                self.commander_handler = CH.LAND

            if (self.commander_handler == CH.LANDING) and (self.th.time()-init_time)>=self.run_duration+2*self.takeoff_and_land_duration:
                break

            # Send takeoff command to all cfs if in takeoff mode
            if self.commander_handler == CH.TAKEOFF:
                self.swarm.allcfs.takeoff(targetHeight=self.flying_height, duration=self.takeoff_and_land_duration)
                self.commander_handler = CH.TAKINGOFF

            # Stop receiving velocity commands and send land command to all cfs in land mode
            if self.commander_handler == CH.LAND:
                for i in range(len(self.swarm.allcfs.crazyflies)):
                    cf = self.swarm.allcfs.crazyflies[i]
                    self.commander(cf.id, stop=True)
                    self.cmd_vel_sub[i].unregister()
                    cf.cmdVelocityWorld([0, 0, 0],0)
                self.swarm.allcfs.land(targetHeight=0.04, duration=self.takeoff_and_land_duration)
                self.commander_handler = CH.LANDING

            # Run the algorithm when in fly mode
            if self.commander_handler == CH.FLY:
                for cf in self.swarm.allcfs.crazyflies:
                    self.commander(cf.id, start=True)
                self.commander_handler = CH.FLYING

            # Sleep for the set rate
            t = Clock()
            #print(self.th.time(), time_s, time_ns)
            t.clock = rospy.Time(self.th.time())
            self.clock_pub.publish(t)
            self.th.sleepForRate(self.rate)
            ii += 1
    
    def vel_callback(self, v:Twist, args):

        cf = args[0]
        cf.cmdVelocityWorld([v.linear.x, v.linear.y, v.linear.z],0)

    def commander(self, id, start:bool = False, stop:bool = False):

        #rospy.wait_for_service('node'+str(i+1)+'/commander')
        try:
            commander = rospy.ServiceProxy('node'+str(id)+'/commander', Commander)
            resp = commander(start = start, stop = stop)
            return resp.status
        except rospy.ServiceException as e:
            #print("Service call failed: %s"%e)
            pass

    def position_publisher(self, pub:rospy.Publisher):

        curr_pos = np.ndarray((len(pub), 3))

        for i in range(len(pub)):
            cf = self.swarm.allcfs.crazyflies[i]

            pose = PoseStamped()
            pos = Pose()
            pos.position.x = cf.position()[0]
            pos.position.y = cf.position()[1]
            pos.position.z = cf.position()[2]
            pos.orientation.w = 0
            pos.orientation.x = 0
            pos.orientation.y = 0
            pos.orientation.z = 1
            pose.pose = pos
            pose.header.stamp = rospy.Time(self.th.time())
            pose.header.frame_id = "map"
            pub[i].publish(pose)
            curr_pos[i] = [cf.position()[0],cf.position()[1],cf.position()[2]]

            # if i == 0:
            #     print(i+1, self.th.time(), int(time_s), int(time_ns), cf.position())

        return curr_pos

    def velocity_publisher(self, pub:rospy.Publisher):

        for i in range(len(pub)):
            time_s, time_d = divmod(self.th.time(), 1)
            time_ns = time_d * 10**9
            cf = self.swarm.allcfs.crazyflies[i]

            vel_stamped = TwistStamped()
            v = Twist()
            v.linear.x = cf.velocity()[0]
            v.linear.y = cf.velocity()[1]
            v.linear.z = cf.velocity()[2]
            v.angular.x = 0
            v.angular.y = 0
            v.angular.z = 0
            vel_stamped.header.stamp.secs = int(time_s)
            vel_stamped.header.stamp.nsecs = int(time_ns)
            vel_stamped.header.frame_id = "map"
            vel_stamped.twist = v
            pub[i].publish(vel_stamped)

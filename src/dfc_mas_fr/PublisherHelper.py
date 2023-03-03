#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Pose
from dfc_mas_fr.srv import Commander, CommanderResponse

class PublisherHelper:

    def __init__(self, swarm):

        self.swarm  = swarm
        self.th = swarm.timeHelper
        self.number_of_agents = rospy.get_param("number_of_agents")
        self.run_duration = rospy.get_param("run_duration")
        self.takeoff_and_land_duration = rospy.get_param("takeoff_and_land_duration")
        self.flying_height = rospy.get_param("flying_height")

    def run_algorithm(self):

        pub_pos = np.ndarray((len(self.swarm.allcfs.crazyflies),),rospy.Publisher)
        pub_vel = np.ndarray((len(self.swarm.allcfs.crazyflies),),rospy.Publisher)

        cmd_vel_sub = np.ndarray((len(self.swarm.allcfs.crazyflies),),rospy.Subscriber)
        curr_pos = np.ndarray((len(pub_pos),),Pose)
        prev_pos = np.ndarray((len(pub_pos),),Pose)
        rate = 30
        first_iteration = True

        for i in range(len(self.swarm.allcfs.crazyflies)):
            cf = self.swarm.allcfs.crazyflies[i]
            pub_pos[i] = rospy.Publisher('node'+str(cf.id)+'/pose', PoseStamped, queue_size=10)
            pub_vel[i] = rospy.Publisher('node'+str(cf.id)+'/twist', TwistStamped, queue_size=10)
            cmd_vel_sub[i] = rospy.Subscriber('node'+str(cf.id)+'/cmd_vel', Twist, self.vel_callback, (cf,))

        # Takeoff
        self.swarm.allcfs.takeoff(targetHeight=self.flying_height, duration=self.takeoff_and_land_duration)
        self.th.sleep(self.takeoff_and_land_duration)

        init_time = self.th.time()
        while (self.th.time()-init_time)<self.run_duration:
            # Publish agent positions
            curr_pos = self.position_publisher(pub_pos, self.swarm)
            if first_iteration:
                prev_pos = curr_pos
            # Publish agent velocities
            self.velocity_publisher(pub_vel, curr_pos, prev_pos, 1/rate)
            prev_pos = curr_pos
            self.th.sleepForRate(rate)
            if first_iteration:
                for cf in self.swarm.allcfs.crazyflies:
                    self.commander(cf.id, start=True)
            first_iteration = False

        # Stop receiving velocity commands
        for i in range(len(self.swarm.allcfs.crazyflies)):
            cf = self.swarm.allcfs.crazyflies[i]
            self.commander(cf.id, stop=True)
            cmd_vel_sub[i].unregister()
            cf.cmdVelocityWorld([0, 0, 0],0)
        self.th.sleep(1)

        # Land
        self.swarm.allcfs.land(targetHeight=0.04, duration=self.takeoff_and_land_duration)
        self.th.sleep(self.takeoff_and_land_duration)
    
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

    def position_publisher(self, pub:rospy.Publisher, swarm):

        curr_pos = np.ndarray((len(pub),),Pose)

        for i in range(len(pub)):
            time_s, time_d = divmod(swarm.timeHelper.time(), 1)
            time_ns = time_d * 10**9

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
            pose.header.stamp.secs = int(time_s)
            pose.header.stamp.nsecs = int(time_ns)
            pose.header.frame_id = "map"
            pub[i].publish(pose)
            curr_pos[i] = pos
        return curr_pos

    def velocity_publisher(self, pub, curr_pos, prev_pos, step):

        for i in range(len(pub)):
            time_s, time_d = divmod(self.th.time(), 1)
            time_ns = time_d * 10**9

            vel_stamped = TwistStamped()
            v = Twist()
            v.linear.x = (curr_pos[i].position.x-prev_pos[i].position.x)/step
            v.linear.y = (curr_pos[i].position.y-prev_pos[i].position.y)/step
            v.linear.z = (curr_pos[i].position.z-prev_pos[i].position.z)/step
            v.angular.x = 0
            v.angular.y = 0
            v.angular.z = 0
            vel_stamped.header.stamp.secs = int(time_s)
            vel_stamped.header.stamp.nsecs = int(time_ns)
            vel_stamped.header.frame_id = "map"
            vel_stamped.twist = v
            pub[i].publish(vel_stamped)
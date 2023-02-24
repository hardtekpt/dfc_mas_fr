#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

def pos_callback(p:PoseStamped, args):
    pub_marker = args[0]
    id = args[1]

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = sys.argv[1]+str(id)
    marker.id = id
    marker.type = Marker.MESH_RESOURCE
    marker.action = 0
    marker.pose.position.x = p.pose.position.x
    marker.pose.position.y = p.pose.position.y
    marker.pose.position.z = p.pose.position.z
    marker.pose.orientation.x = p.pose.orientation.x
    marker.pose.orientation.y = p.pose.orientation.y
    marker.pose.orientation.z = p.pose.orientation.z
    marker.pose.orientation.w = p.pose.orientation.w
    marker.scale.x = 3
    marker.scale.y = 3
    marker.scale.z = 3
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.mesh_resource = "package://dfc_mas_fr/meshes/crazyflie2.dae"
    marker.mesh_use_embedded_materials = True
    pub_marker.publish(marker)

if __name__ == '__main__':

    number_of_agents = int(sys.argv[2])
    pubs = np.ndarray((number_of_agents,),rospy.Publisher)

    rospy.init_node('marker_publisher')

    for i in range(number_of_agents):
        pub_topic = sys.argv[1]+str(i+1)+'/marker'
        pubs[i] = rospy.Publisher(pub_topic, Marker, queue_size=10)

        rospy.Subscriber('/algorithm/'+sys.argv[1]+str(i+1)+'/pose', PoseStamped, pos_callback, ( pubs[i], i+1 ))

    rospy.spin()
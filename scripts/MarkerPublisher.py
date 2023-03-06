#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from dfc_mas_fr.srv import Commander, Map, MapResponse
from dfc_mas_fr.GradientMap import GradientMap
from dfc_mas_fr.msg import Hiperboloid, Obstacle
import matplotlib
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

def publish_map(pub:rospy.Publisher, map:MapResponse):

    # convert map srv into map object
    map_obj = GradientMap(dimensions=map.map_dimensions)
    map_obj.convert_srv_to_obj(map_srv=map)

    # discretize map 
    res = 0.75
    discrete_map = map_obj.discretize_map(res)

    # for each cell get color, create marker and add marker to marker array
    marker_array = MarkerArray()

    for i in range(discrete_map.shape[0]):
        for j in range(discrete_map.shape[1]):
            x = i * res
            y = j * res
            cmap = matplotlib.cm.get_cmap('viridis')
            rgba = cmap(discrete_map[i,j]/np.max(discrete_map))

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "map"
            marker.id = (i * discrete_map.shape[1])+j
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = res
            marker.scale.y = res
            marker.scale.z = 0.01
            marker.color.a = rgba[3]
            marker.color.r = rgba[0]
            marker.color.g = rgba[1]
            marker.color.b = rgba[2]
            
            marker_array.markers.append(marker)

    # publish marker array
    pub.publish(marker_array)
    
    pass

if __name__ == '__main__':

    number_of_agents = int(sys.argv[2])
    pubs = np.ndarray((number_of_agents,),rospy.Publisher)

    rospy.init_node('marker_publisher')

    for i in range(number_of_agents):
        pub_topic = sys.argv[1]+str(i+1)+'/marker'
        pubs[i] = rospy.Publisher(pub_topic, Marker, queue_size=10)

        rospy.Subscriber('/algorithm/'+sys.argv[1]+str(i+1)+'/pose', PoseStamped, pos_callback, ( pubs[i], i+1 ))

    map_pub = rospy.Publisher('/algorithm/map', MarkerArray, queue_size=10)
    rospy.wait_for_service('publisher/map_handler')
    try:
        get_map = rospy.ServiceProxy('publisher/map_handler', Map)
        resp = get_map()
        rospy.sleep(1)
        publish_map(map_pub, resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.spin()
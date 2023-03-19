#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from dfc_mas_fr.GradientMap import GradientMap
import matplotlib
import numpy as np
from dfc_mas_fr.msg import Map

def pos_circle(p:PoseStamped, args):

    pub = args[0]
    id = args[1]

    cf_radius = rospy.get_param("cf_radius")
    noise_std = rospy.get_param("noise_std")

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = sys.argv[1]+str(id)
    marker.id = id
    marker.type = Marker.CYLINDER
    marker.action = 0
    marker.pose.position.x = p.pose.position.x
    marker.pose.position.y = p.pose.position.y
    marker.pose.position.z = p.pose.position.z
    marker.pose.orientation.x = p.pose.orientation.x
    marker.pose.orientation.y = p.pose.orientation.y
    marker.pose.orientation.z = p.pose.orientation.z
    marker.pose.orientation.w = p.pose.orientation.w
    marker.scale.x = 2*(cf_radius + 3 * noise_std)
    marker.scale.y = 2*(cf_radius + 3 * noise_std)
    marker.scale.z = 0.05
    marker.color.a = 0.6
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    pub.publish(marker)


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
    marker.scale.x = 2
    marker.scale.y = 2
    marker.scale.z = 2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.mesh_resource = "package://dfc_mas_fr/meshes/crazyflie2.dae"
    marker.mesh_use_embedded_materials = True
    pub_marker.publish(marker)

def publish_map(pub:rospy.Publisher, map:Map):

    # convert map msg into map object
    map_obj = GradientMap(dimensions=map.map_dimensions)
    map_obj.convert_msg_to_obj(map)

    # discretize map 
    res = 0.8
    discrete_map = map_obj.discretize_map(res)

    # for each cell get color, create marker and add marker to marker array
    marker_array = MarkerArray()

    for i in range(discrete_map.shape[0]):
        for j in range(discrete_map.shape[1]):
            x = (i+0.5) * res
            y = (j+0.5) * res
            #print(x,y,i,j)
            cmap = matplotlib.cm.get_cmap('viridis')
            map_norm = (discrete_map[i,j]-np.min(discrete_map))/(np.max(discrete_map)-np.min(discrete_map))
            rgba = cmap(map_norm)

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

    for i in range(len(map_obj.map['obstacles'])):
        obs = map_obj.map['obstacles'][i]

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "map_obs"
        marker.id = discrete_map.shape[0] * discrete_map.shape[1] + i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obs['pos'][0] + obs['size'][0]/2
        marker.pose.position.y = obs['pos'][1] + obs['size'][1]/2
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = obs['size'][0]
        marker.scale.y = obs['size'][1]
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0
        
        marker_array.markers.append(marker)

    # publish marker array
    pub.publish(marker_array)
    
def map_update(update_msg:Map, args):

    publish_map(map_pub, update_msg)


if __name__ == '__main__':

    number_of_agents = rospy.get_param("number_of_agents")
    pubs = np.ndarray((number_of_agents,),rospy.Publisher)
    pubs_circle = np.ndarray((number_of_agents,),rospy.Publisher)

    rospy.init_node('marker_publisher')

    for i in range(number_of_agents):
        pub_topic = sys.argv[1]+str(i+1)+'/marker'
        pubs[i] = rospy.Publisher(pub_topic, Marker, queue_size=10)
        pubs_circle[i] = rospy.Publisher(pub_topic+'_circle', Marker, queue_size=10)

        rospy.Subscriber('/algorithm/'+sys.argv[1]+str(i+1)+'/pose', PoseStamped, pos_callback, ( pubs[i], i+1 ))
        rospy.Subscriber('/algorithm/'+sys.argv[1]+str(i+1)+'/pose_estimate', PoseStamped, pos_circle, ( pubs_circle[i], i+1 ))

    map_pub = rospy.Publisher('/algorithm/map', MarkerArray, queue_size=10)
    rospy.Subscriber('publisher/map', Map, map_update, (map_pub, ))

    # rospy.wait_for_service('publisher/map_handler')
    
    # try:
    #     get_map = rospy.ServiceProxy('publisher/map_handler', Map)
    #     resp = get_map()
    #     map = GradientMap(dimensions=resp.map_dimensions)
    #     map.convert_srv_to_obj(resp)
        
    #     rospy.sleep(1)
    #     publish_map(map_pub, resp)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)

    rospy.spin()
#!/usr/bin/env python3

import rospy
import numpy as np
from dfc_mas_fr.msg import MapUpdate
from dfc_mas_fr.GradientMap import GradientMap
from dfc_mas_fr.srv import Map

class MapUpdateHelper:

    def __init__(self):

        self.map_update_pub = rospy.Publisher('publisher/map_update', MapUpdate, queue_size=10)
        self.map = self.init_map()

        rospy.Service('publisher/map_handler', Map, self.handle_map_srv)

    def init_map(self):
        
        map_dimensions = np.asarray([20, 20])
        map = GradientMap(dimensions=map_dimensions)

        hiperboloid_center = np.asarray([10, 10])
        hiperboloid_params = np.asarray([3000, 2, 2, 15])
        map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

        return map
    
    def handle_map_srv(self, req):
        map_msg = self.map.convert_obj_to_srv()
        return map_msg
    
    def map_update_translation_test(self):

        msg = MapUpdate()
        msg.operation = 'translation'
        msg.idx = 0
        msg.hip.center = [15, 15]

        self.map_update_pub.publish(msg)
#!/usr/bin/env python3

import rospy
import numpy as np
from dfc_mas_fr.msg import Map
from dfc_mas_fr.GradientMap import GradientMap
import random
from enum import Enum

class MapUpdateHelper:

    MAX_DISTANCE_PER_TIME_STEP = 0.5

    VISIT_MULTIPLIER = 0.01

    INSPECTION_THERESHOLD = 800

    MAP_TYPE = Enum('Map_Type', ['SINGLE_MAX', 'SINGLE_MIN', 'MULTIPLE_MAX_AND_MIN'])

    def __init__(self):

        self.map_update_pub = rospy.Publisher('publisher/map', Map, queue_size=10)
        
        self.obstacles = False
        self.dynamic_pos = False
        self.dynamic_val = False
        self.random_creation_and_destruction_of_hiperboloids = False
        self.map_type = MapUpdateHelper.MAP_TYPE.SINGLE_MAX

        self.map = self.init_map()

    def init_map(self):
        
        map_dimensions = np.asarray([20, 20])
        map = GradientMap(dimensions=map_dimensions)

        if self.map_type == MapUpdateHelper.MAP_TYPE.SINGLE_MAX or self.map_type == MapUpdateHelper.MAP_TYPE.SINGLE_MIN:
            hiperboloid_center = np.asarray([10, 10])
            hiperboloid_params = np.asarray([3000.0, 2, 2, 15])
            if self.map_type == MapUpdateHelper.MAP_TYPE.SINGLE_MIN:
                hiperboloid_params = np.asarray([-3000, 2, 2, 15])
            map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params, d_pos=self.dynamic_pos, d_val=self.dynamic_val)

        if self.map_type == MapUpdateHelper.MAP_TYPE.MULTIPLE_MAX_AND_MIN:
            hiperboloid_center = np.asarray([2, 5])
            hiperboloid_params = np.asarray([2000.0, 3, 3, 15])
            map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params, d_pos=self.dynamic_pos, d_val=self.dynamic_val)

            hiperboloid_center = np.asarray([17, 15])
            hiperboloid_params = np.asarray([2000.0, 3, 3, 15])
            map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params, d_pos=self.dynamic_pos, d_val=self.dynamic_val)

            hiperboloid_center = np.asarray([16, 4])
            hiperboloid_params = np.asarray([-2000.0, 3, 3, 15])
            map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params, d_pos=self.dynamic_pos, d_val=self.dynamic_val)

            hiperboloid_center = np.asarray([8, 18])
            hiperboloid_params = np.asarray([-2000.0, 3, 3, 15])
            map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params, d_pos=self.dynamic_pos, d_val=self.dynamic_val)

        if self.obstacles:
            map.add_obstacle(position=[8.7, 17])
            map.add_obstacle(position=[2.8, 16.4])
            map.add_obstacle(position=[7.4, 10.4])
            map.add_obstacle(position=[17, 4.3])
            map.add_obstacle(position=[2.2, 7.8])
            map.add_obstacle(position=[6.1, 17.3])
            map.add_obstacle(position=[1, 17])
            map.add_obstacle(position=[13.1, 11.6])
            map.add_obstacle(position=[16.8, 6.7])
            map.add_obstacle(position=[13.2, 2.4])

        return map
    
    def map_publisher(self, curr_pos):

        if self.random_creation_and_destruction_of_hiperboloids:
            if np.random.randint(0, 10) == 0 and len(self.map.map['hiperboloids']) > 1:
                idx = np.random.randint(0, len(self.map.map['hiperboloids']))
                self.map.rm_hiperboloid(idx)
            
            if np.random.randint(0, 10) == 0:
                hiperboloid_center = np.asarray([np.random.rand()*20, np.random.rand()*20])
                hiperboloid_params = np.asarray([[-1,1][random.randrange(2)]*2000, 3, 3, 15])
                self.map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params, d_pos=self.dynamic_pos, d_val=self.dynamic_val) 

        h=0
        while h < len(self.map.map['hiperboloids']):
            if self.map.map['hiperboloids'][h]['d_pos']:
                self.get_new_pos(h) 
            if self.map.map['hiperboloids'][h]['d_val']:
                self.get_new_val(h, curr_pos) 
                if np.abs(self.map.map['hiperboloids'][h]['params'][0]) < MapUpdateHelper.INSPECTION_THERESHOLD:
                    self.map.rm_hiperboloid(h)  
                    h -= 1
            h += 1            
        msg = Map()
        msg = self.map.convert_obj_to_msg()
        self.map_update_pub.publish(msg)

    def get_new_pos(self, h):

        head = np.random.normal(0.2,1,2)
        head /= np.linalg.norm(head)
        self.map.map['hiperboloids'][h]['center'] =  self.map.map['hiperboloids'][h]['center'] + head * MapUpdateHelper.MAX_DISTANCE_PER_TIME_STEP

    def get_new_val(self, h, curr_pos):
        for p in curr_pos:
            if self.map.check_if_agent_is_in_rendezvous_area(p[0:2], h):
                self.map.map['hiperboloids'][h]['params'][0] -= self.map.map['hiperboloids'][h]['params'][0] * MapUpdateHelper.VISIT_MULTIPLIER

    

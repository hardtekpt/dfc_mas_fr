#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import csv
from dfc_mas_fr.msg import Hiperboloid, Obstacle, Map
class GradientMap():

    OBSTACLE_UTILITY_VALUE = -2000

    DRAW_RESOLUTION = 0.05

    RENDEZVOUS_AREA_INSPECTION_MINIMUM_DISTANCE = 2

    def __init__(self, dimensions:np.ndarray((2,)) = np.asarray([4, 7])):
        
        self.dimensions = dimensions
        self.map = {'hiperboloids': [], 'obstacles': []}

    def add_hiperboloid(self, center:np.ndarray((2,)), params:np.ndarray((4,)), d_pos:bool = False, d_val:bool = False):

        hiperboloid = {'center': center, 'params': params, 'd_pos':d_pos, 'd_val': d_val}
        self.map['hiperboloids'].append(hiperboloid)

    def add_obstacle(self, position:np.ndarray((2,)) = np.asarray([0.0, 0.0]), size:np.ndarray((2,)) = np.asarray([2, 2]), random:bool = False):
        
        if random:
            position[0] = np.random.uniform(0,self.dimensions[0] - size[0])
            position[1] = np.random.uniform(0,self.dimensions[1] - size[1])
        
        obstacle = {'pos': position, 'size': size}
        self.map['obstacles'].append(deepcopy(obstacle))

    def show(self, block:bool = True):

        grid_dimensions = np.ndarray((2,))
        grid_dimensions[0] = self.dimensions[0] / GradientMap.DRAW_RESOLUTION
        grid_dimensions[1] = self.dimensions[1] / GradientMap.DRAW_RESOLUTION

        discrete_map = self.discretize_map(GradientMap.DRAW_RESOLUTION)

        for obstacle in self.map['obstacles']:
            position_indxs = [int(obstacle['pos'][0]/GradientMap.DRAW_RESOLUTION), int(obstacle['pos'][1]/GradientMap.DRAW_RESOLUTION)]
            size_indxs = [int(obstacle['size'][0]/GradientMap.DRAW_RESOLUTION), int(obstacle['size'][1]/GradientMap.DRAW_RESOLUTION)]

            for y_idx in range(position_indxs[1], np.min([position_indxs[1] + size_indxs[1], grid_dimensions[1]]).astype(int)):
                discrete_map[y_idx][position_indxs[0]: np.min([position_indxs[0]+size_indxs[0], grid_dimensions[0]]).astype(int)] = GradientMap.OBSTACLE_UTILITY_VALUE

        color_map = plt.get_cmap('viridis')
        color_map.set_bad(color='black')

        plt.figure("Gradient Map")
        masked_array = np.ma.masked_where(discrete_map == GradientMap.OBSTACLE_UTILITY_VALUE, discrete_map)
        plt.imshow(masked_array, cmap=color_map)

        plt.xlabel("x axis")
        plt.ylabel("y axis")
        ax = plt.gca()
        ax.set_xticks(np.arange(0, discrete_map.shape[1], 10))
        ax.set_xticklabels(np.arange(0, discrete_map.shape[1], 10)*GradientMap.DRAW_RESOLUTION)
        ax.set_yticks(np.arange(0, discrete_map.shape[0], 10))
        ax.set_yticklabels(np.arange(0, discrete_map.shape[0], 10)*GradientMap.DRAW_RESOLUTION)
        plt.colorbar()
        if block:
            plt.show()

    def export(self, name:str):

        with open(name, 'w', newline='') as file:
            writer = csv.writer(file, delimiter=',')

            writer.writerow([self.dimensions[0], self.dimensions[1], len(self.map['hiperboloids']), len(self.map['obstacles'])])

            for hiperboloid in self.map['hiperboloids']:
                writer.writerow([hiperboloid['center'][0], hiperboloid['center'][1], hiperboloid['params'][0], hiperboloid['params'][1], hiperboloid['params'][2], hiperboloid['params'][3]])
            
            for obstacle in self.map['obstacles']:
                writer.writerow([obstacle['pos'][0], obstacle['pos'][1], obstacle['size'][0], obstacle['size'][1]])

    def import_map(self, name:str):

        with open(name, 'r', newline='') as file:
            reader = csv.reader(file, delimiter=',')
            line_count = 0
            hiperboloid_n = 0
            obstacle_n = 0
            for row in reader:
                if line_count == 0:
                    self.dimensions[0] = float(row[0])
                    self.dimensions[1] = float(row[1])
                    hiperboloid_n = int(row[2])
                    obstacle_n = int(row[3])
                elif line_count <= hiperboloid_n:
                    self.add_hiperboloid(center=np.asarray([float(row[0]), float(row[1])]), params=np.asarray([float(row[2]), float(row[3]), float(row[4]), float(row[5])]))
                elif line_count <= obstacle_n + hiperboloid_n:
                    self.add_obstacle(position=np.asarray([float(row[0]), float(row[1])]), size=np.asarray([float(row[2]), float(row[3])]))
                line_count += 1

    def rm_hiperboloid(self, hiperboloid_idx:int):
        
        del self.map['hiperboloids'][hiperboloid_idx]

    def get_utility_value(self, position:np.ndarray((2,))):
        
        value = 0
        for hiperboloid in self.map['hiperboloids']:
            k = hiperboloid['params'][0]
            theta = hiperboloid['params'][1]
            thau = hiperboloid['params'][2]
            w = hiperboloid['params'][3]
            value += k / ((theta * (position[0] - hiperboloid['center'][1]))**2 + (thau * (position[1] - hiperboloid['center'][0]))**2 + w)
        
        for obstacle in self.map['obstacles']:
            if self.check_for_obstacle_collision(position, obstacle, 0):
                value == GradientMap.OBSTACLE_UTILITY_VALUE

        return value

    def get_gradient(self, position:np.ndarray((2,))):

        dx = 0
        dy = 0

        for hiperboloid in self.map['hiperboloids']:
            k = hiperboloid['params'][0]
            theta = hiperboloid['params'][1]
            thau = hiperboloid['params'][2]
            w = hiperboloid['params'][3]
            x = position[0] - hiperboloid['center'][1]
            y = position[1] - hiperboloid['center'][0]
        
            dx += (-2*k*(theta**2)*x)/(((theta**2)*(x**2) + (thau**2)*(y**2) + w)**2)
            dy += (-2*k*(thau**2)*y)/(((theta**2)*(x**2) + (thau**2)*(y**2) + w)**2)

        return np.asarray([dx, dy])

    def check_if_agent_is_in_rendezvous_area(self, position:np.ndarray((2,)), i):

        hip = np.asarray([self.map['hiperboloids'][i]['center'][1], self.map['hiperboloids'][i]['center'][0]])

        if np.linalg.norm(position - hip) <= GradientMap.RENDEZVOUS_AREA_INSPECTION_MINIMUM_DISTANCE:
            return True
        return False

    def check_for_obstacle_collision(self, position:np.ndarray((2,)), obstacle, margin):

        if (position[0] >= obstacle['pos'][0] - margin) and (position[0] <= obstacle['pos'][0] + obstacle['size'][0] + margin):
            if (position[1] >= obstacle['pos'][1] - margin) and (position[1] <= obstacle['pos'][1] + obstacle['size'][1] + margin):
                return True
            
        corners = np.array([[obstacle['pos'][0], obstacle['pos'][1]], 
                   [obstacle['pos'][0] + obstacle['size'][0], obstacle['pos'][1]], 
                   [obstacle['pos'][0], obstacle['pos'][1] + obstacle['size'][1]], 
                   [obstacle['pos'][0] + obstacle['size'][0], obstacle['pos'][1] + obstacle['size'][1]]])
        
        for c in corners:
            if np.linalg.norm(c - position) <= margin:
                return True
            
        return False
    
    def convert_obj_to_msg(self):
        
        map_msg = Map()
        map_msg.map_dimensions = self.dimensions
        map_msg.n_obs = 0
        map_msg.n_hip = 0

        for i in range(len(self.map['hiperboloids'])):
            hip_msg = Hiperboloid()
            hip = self.map['hiperboloids'][i]

            map_msg.n_hip += 1
            hip_msg.center = hip['center']
            hip_msg.params = hip['params']
            map_msg.hiperboloids.append(hip_msg)

        for i in range(len(self.map['obstacles'])):
            obs_msg = Obstacle()
            obs = self.map['obstacles'][i]

            map_msg.n_obs += 1
            obs_msg.pos = obs['pos']
            obs_msg.size = obs['size']
            map_msg.obstacles.append(obs_msg)
        return map_msg

    def convert_msg_to_obj(self, map_msg):
    
        for i in range(map_msg.n_hip):
            hip = Hiperboloid()
            hip = map_msg.hiperboloids[i]
            self.add_hiperboloid(hip.center, hip.params)

        for i in range(map_msg.n_obs):
            obs = Obstacle()
            obs = map_msg.obstacles[i]
            self.add_obstacle(obs.pos, obs.size)

    def discretize_map(self, res): 

        x = np.linspace(0, self.dimensions[1], int(self.dimensions[1]/res))
        y = np.linspace(0, self.dimensions[0], int(self.dimensions[0]/res))

        discrete_map = np.zeros((len(x),len(y)))
        grid_dimensions = np.ndarray((2,))
        grid_dimensions[0] = self.dimensions[0] / res
        grid_dimensions[1] = self.dimensions[1] / res

        for hiperboloid in self.map['hiperboloids']:
            for x_idx in range(len(x)):
                for y_idx in range(len(y)):
                    k = hiperboloid['params'][0]
                    theta = hiperboloid['params'][1]
                    thau = hiperboloid['params'][2]
                    w = hiperboloid['params'][3]
                    discrete_map[x_idx][y_idx] += k / ((theta * (x[x_idx] - hiperboloid['center'][1]))**2 + (thau * (y[y_idx] - hiperboloid['center'][0]))**2 + w)

        # for obstacle in self.map['obstacles']:
        #     position_indxs = [int(obstacle['pos'][0]/res), int(obstacle['pos'][1]/res)]
        #     size_indxs = [int(obstacle['size'][0]/res), int(obstacle['size'][1]/res)]

        #     for y_idx in range(position_indxs[1], np.min([position_indxs[1] + size_indxs[1], grid_dimensions[1]]).astype(int)):
        #         discrete_map[y_idx][position_indxs[0]: np.min([position_indxs[0]+size_indxs[0], grid_dimensions[0]]).astype(int)] = GradientMap.OBSTACLE_UTILITY_VALUE

        return discrete_map

    def map_update(self, map:Map):

        self.dimensions = map.map_dimensions
        self.map = {'hiperboloids': [], 'obstacles': []}
        self.convert_msg_to_obj(map)
        

if __name__ == "__main__":

    map_dimensions = np.asarray([20, 20])
    map = GradientMap(dimensions=map_dimensions)

    hiperboloid_center = np.asarray([10, 10])
    hiperboloid_params = np.asarray([3000.0, 2, 2, 15])
    map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)   

    print(map.map['obstacles'])                                

    map.show()

    #hiperboloid_center = np.asarray([1, 1])
    #hiperboloid_params = np.asarray([3000, 10, 10, 15])
    #map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

    #map.add_obstacle(position=np.asarray([1,2]), size=np.asarray([0.5, 1]))

    #map.add_obstacle(random=True)
    #map.add_obstacle(random=True)
    #map.add_obstacle(random=True)

    #map.add_obstacle(random=True, size=np.asarray([0.2,1.5]))

    #map.export(name="test_file.csv")
    #map.show()

    #map2 = GradientMap()
    #map2.import_map(name="test_file.csv")
    #map2.rm_hiperboloid(0)
    #map2.show()


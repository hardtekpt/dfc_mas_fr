#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import csv
class GradientMap():

    OBSTACLE_UTILITY_VALUE = -2000

    DRAW_RESOLUTION = 0.05

    RENDEZVOUS_AREA_INSPECTION_MINIMUM_DISTANCE = 1

    def __init__(self, dimensions:np.ndarray((2,)) = np.asarray([4, 7])):
        
        self.dimensions = dimensions
        self.map = {'hiperboloids': [], 'obstacles': []}

    def add_hiperboloid(self, center:np.ndarray((2,)), params:np.ndarray((4,))):

        hiperboloid = {'center': center, 'params': params}
        self.map['hiperboloids'].append(hiperboloid)

    def add_obstacle(self, position:np.ndarray((2,)) = np.asarray([0.0, 0.0]), size:np.ndarray((2,)) = np.asarray([0.5, 0.5]), random:bool = False):
        
        if random:
            position[0] = np.random.uniform(0,self.dimensions[0] - size[0])
            position[1] = np.random.uniform(0,self.dimensions[1] - size[1])
        
        obstacle = {'pos': position, 'size': size}
        self.map['obstacles'].append(deepcopy(obstacle))

    def show(self, block:bool = True):
        
        x = np.linspace(0, self.dimensions[1], int(self.dimensions[1]/GradientMap.DRAW_RESOLUTION))
        y = np.linspace(0, self.dimensions[0], int(self.dimensions[0]/GradientMap.DRAW_RESOLUTION))

        discrete_map = np.zeros((len(x),len(y)))
        grid_dimensions = self.dimensions / GradientMap.DRAW_RESOLUTION

        for hiperboloid in self.map['hiperboloids']:
            for x_idx in range(len(x)):
                for y_idx in range(len(y)):
                    k = hiperboloid['params'][0]
                    theta = hiperboloid['params'][1]
                    thau = hiperboloid['params'][2]
                    w = hiperboloid['params'][3]
                    discrete_map[x_idx][y_idx] += k / ((theta * (x[x_idx] - hiperboloid['center'][1]))**2 + (thau * (y[y_idx] - hiperboloid['center'][0]))**2 + w)

        for obstacle in self.map['obstacles']:
            position_indxs = (obstacle['pos']/GradientMap.DRAW_RESOLUTION).astype(int)
            size_indxs = (obstacle['size']/GradientMap.DRAW_RESOLUTION).astype(int)

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
            if self.check_for_obstacle_collision(position, obstacle):
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
        
            dx += - (2*k*(theta**2)*position[0])/(((theta**2)*(position[0]**2) + (thau**2)*(position[1]**2) + w)**2)
            dy += - (2*k*(thau**2)*position[1])/(((theta**2)*(position[0]**2) + (thau**2)*(position[1]**2) + w)**2)

        return np.asarray([dx, dy])

    def check_if_agent_is_in_rendezvous_area(self, position:np.ndarray((2,))):

        for i in range(len(self.map['hiperboloids'])):
            if np.linalg.norm(position - self.map['hiperboloids'][i]['pos']) <= GradientMap.RENDEZVOUS_AREA_INSPECTION_MINIMUM_DISTANCE:
                return i
        return -1

    def check_for_obstacle_collision(position:np.ndarray((2,)), obstacle):

        if (position[0] >= obstacle['pos'][0]) and (position[0] <= obstacle['pos'][0] + obstacle['size'][0]):
            if (position[1] >= obstacle['pos'][1]) and (position[1] <= obstacle['pos'][1] + obstacle['size'][1]):
                return True
        return False
        

if __name__ == "__main__":

    map_dimensions = np.asarray([4, 7])
    map = GradientMap(dimensions=map_dimensions)

    hiperboloid_center = np.asarray([2, 3.5])
    hiperboloid_params = np.asarray([5000, 3, 3, 15])
    map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

    map.show()

    hiperboloid_center = np.asarray([1, 1])
    hiperboloid_params = np.asarray([3000, 10, 10, 15])
    map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

    map.add_obstacle(position=np.asarray([1,2]), size=np.asarray([0.5, 1]))

    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)

    map.add_obstacle(random=True, size=np.asarray([0.2,1.5]))

    #map.export(name="test_file.csv")
    #map.show()

    map2 = GradientMap()
    map2.import_map(name="test_file.csv")
    #map2.rm_hiperboloid(0)
    map2.show()


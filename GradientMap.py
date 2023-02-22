#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
class GradientMap():

    def __init__(self, dimensions:np.ndarray((2,)) = np.asarray([4, 7])):
        
        self.dimensions = dimensions
        self.map = {'hiperboloid': [], 'obstacles': []}

    def add_hiperboloid(self, center:np.ndarray((2,)), params:np.ndarray((4,))):

        hiperboloid = {'center': center, 'params': params}
        self.map['hiperboloid'].append(hiperboloid)

    def add_obstacle(self, position:np.ndarray((2,)) = np.asarray([0, 0]), size:np.ndarray((2,)) = np.asarray([0.5, 0.5]), random:bool = False):
        
        obstacle = {'pos': position, 'size': size}
        self.map['obstacles'].append(obstacle)

    def show(self):

        color_map = plt.get_cmap('viridis')
        color_map.set_bad(color='black')

        masked_array = np.ma.masked_where(self.map == -2000, self.map)
        plt.imshow(masked_array, cmap=color_map)

        plt.xlabel("x axis")
        plt.ylabel("y axis")
        ax = plt.gca()
        ax.set_xticks(np.arange(0, self.map.shape[1], 5))
        ax.set_xticklabels(np.arange(0, self.map.shape[1], 5)*self.grid_size)
        ax.set_yticks(np.arange(0, self.map.shape[0], 5))
        ax.set_yticklabels(np.arange(0, self.map.shape[0], 5)*self.grid_size)
        plt.colorbar()
        plt.show()

    def export(self, name:str):
        pass

    def import_map(self, name:str):
        pass

    def rm_hiperboloid(self):
        pass

    def get_utility_value(self):
        pass

    def get_gradient(self):
        pass

    def check_if_agent_is_in_rendezvous_area(self):
        pass

        

if __name__ == "__main__":

    map_dimensions = np.asarray([4, 7])
    map = GradientMap(dimensions=map_dimensions)

    map.show()
#!/usr/bin/env python3

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

class GradientMapGenerator:

    INITIAL_VALUE = -2000

    def __init__(self, map_dimensions:np.ndarray((2,)), grid_size:float):

        self.map_dimensions = map_dimensions
        self.grid_size = grid_size

        self.grid_dimensions = self.map_dimensions / self.grid_size
        self.map = np.full((self.grid_dimensions[1].astype(int),self.grid_dimensions[0].astype(int)), GradientMapGenerator.INITIAL_VALUE, dtype = float)
    
    def add_hiperboloid(self, center:np.ndarray((2,)), params:np.ndarray((4,))):
        for x_idx in range(self.map.shape[0]):
            for y_idx in range(self.map.shape[1]):
                x = x_idx * self.grid_size - center[1]
                y = y_idx * self.grid_size - center[0]

                k = params[0]
                theta = params[1]
                thau = params[2]
                w = params[3]

                h = k / ((theta * x)**2 + (thau * y)**2 + w)
                
                self.map[x_idx][y_idx] += h

    def add_obstacle(self, position:np.ndarray((2,)) = np.asarray([0, 0]), size:np.ndarray((2,)) = np.asarray([0.5, 0.5]), random:bool = False):

        position_indxs = (position/self.grid_size).astype(int)
        size_indxs = (size/self.grid_size).astype(int)

        if random:
            position_indxs[0] = np.random.randint(0, self.map_dimensions[0]/self.grid_size)
            position_indxs[1] = np.random.randint(0, self.map_dimensions[1]/self.grid_size)

        for y_idx in range(position_indxs[1], np.min([position_indxs[1] + size_indxs[1], self.grid_dimensions[1]]).astype(int)):
            self.map[y_idx][position_indxs[0]: np.min([position_indxs[0]+size_indxs[0], self.grid_dimensions[0]]).astype(int)] = -2000


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

        map_params = str(self.map_dimensions[0]) + ',' + str(self.map_dimensions[1]) + ',' + str(self.grid_size)

        np.savetxt(name+".csv", self.map, delimiter=",", header=map_params)


if __name__ == "__main__":

    map_dimensions = np.asarray([4, 7])
    grid_size = 0.2
    map = GradientMapGenerator(map_dimensions=map_dimensions, grid_size=grid_size)

    hiperboloid_center = np.asarray([2, 3.5])
    hiperboloid_params = np.asarray([5000, 3, 3, 15])
    map.add_hiperboloid(center=hiperboloid_center, params=hiperboloid_params)

    obs_pos = np.asarray([0,0])
    obs_size = np.asarray([0.5,0.5])
    map.add_obstacle(size=obs_size, position=obs_pos)

    map.add_obstacle(random=True)
    map.add_obstacle(random=True)
    map.add_obstacle(random=True)

    map.add_obstacle(random=True, size=np.asarray([0.2,1.5]))

    map.show()

    #map.export(name="test_map")
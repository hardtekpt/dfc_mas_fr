#!/usr/bin/env python3

import numpy as np

class GradientMap():

    def __init__(self, file:str):
        f = open(file, "r")
        params = f.readline()
        f.close()
        params = params.split(" ")[1]

        self.map_dimensions = np.asarray([float(params.split(",")[0]), float(params.split(",")[1])])
        self.grid_size = float(params.split(",")[2])

        self.grid_dimensions = self.map_dimensions / self.grid_size
        self.map = np.loadtxt(fname=file, comments=None, skiprows=1, delimiter=',')

    def add_hiperboloid(self, center:np.ndarray((2,)), params:np.ndarray((4,))):
        pass

    def add_obstacle(self, position:np.ndarray((2,)) = np.asarray([0, 0]), size:np.ndarray((2,)) = np.asarray([0.5, 0.5]), random:bool = False):
        pass

    def show(self):
        pass

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

    map = GradientMap(file="test_map.csv")

    map.show()
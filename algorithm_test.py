#!/usr/bin/env python3

import numpy as np
import sys
from os.path import dirname, abspath
sys.path.insert(0, dirname(dirname(abspath(__file__))))

from pycrazyswarm import *
import matplotlib.pyplot as plt


class dfc_mas_fr:
    def __init__(self, number_of_agents: int, sensing_radius: float):
        self.number_of_agents = number_of_agents
        self.sensing_radius = sensing_radius

    def run(self):
        # run the algorithm, calculate heading for each vehicle each time step
        pass

    def update_movement(self, agent_index: int, current_positions: np.ndarray, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):
        
        separation = self.get_separation_vector(agent_index, current_positions, distribution)


        heading = 0
        
        return heading
    
    def get_separation_vector(self, agent_index: int, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):

        neighbours = np.where(distribution == 1)[0]
        distance = 0

        for neighbour in neighbours:
            distance += current_positions[agent_index, :] - current_positions[neighbour, :]

        separation = distance/len(neighbours)
        
        return separation
    
    def get_cohesion_vector(self, agent_index: int, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):
    
        neighbours = np.where(distribution == 1)[0]
        distance = 0

        for neighbour in neighbours:
            distance += current_positions[neighbour, :] - current_positions[agent_index, :]

        cohesion = distance/len(neighbours)
        
        return cohesion
    
    def get_alignment_vector(self, agent_index: int, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):
        
        neighbours = np.where(distribution == 1)[0]
        distance = 0

        for neighbour in neighbours:
            distance += current_velocities[neighbour, :] - current_velocities[agent_index, :]

        alignment = distance/len(neighbours)
        
        return alignment
             
    


Z = 1.0
sleepRate = 30


def goCircle(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cf.position()
        startPos = cf.initialPosition + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        while timeHelper.time() - startTime < 10:
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
            timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":

    crazyflies_yaml = "./launch/crazyflies.yaml"

    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #algorithm = dfc_mas_fr(number_of_agents=10, sensing_radius=5)
    #algorithm.create_utility_function()

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=4, radius=1, kPosition=1)
    allcfs.land(targetHeight=0.04, duration=3)
    timeHelper.sleep(3)

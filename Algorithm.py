#!/usr/bin/env python3

import numpy as np
from GradientMap import GradientMap

class Algorithm:
    def __init__(self, number_of_agents: int, sensing_radius: float, map: GradientMap):
        self.number_of_agents = number_of_agents
        self.sensing_radius = sensing_radius
        self.map = map

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
        difference = 0

        for neighbour in neighbours:
            difference += current_velocities[neighbour, :] - current_velocities[agent_index, :]

        alignment = difference/len(neighbours)
        
        return alignment
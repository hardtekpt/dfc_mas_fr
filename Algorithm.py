#!/usr/bin/env python3

import numpy as np
from GradientMap import GradientMap
import math

class Algorithm:
    def __init__(self, number_of_agents: int, sensing_radius: float, map: GradientMap, control_weights: np.ndarray((6,))):
        self.number_of_agents = number_of_agents
        self.sensing_radius = sensing_radius
        self.map = map
        self.control_weights = control_weights

    def run(self):
        # run the algorithm, calculate heading for each vehicle each time step
        pass

    def update_movement(self, agent_index: int, current_positions: np.ndarray, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):

        cw = self.control_weights
        
        separation = self.get_separation_vector(agent_index, current_positions, distribution)
        cohesion = self.get_cohesion_vector(agent_index,current_positions,distribution)
        alignment = self.get_alignment_vector(agent_index,current_velocities,distribution)
        attraction = self.get_attraction_vector(agent_index,current_positions,distribution)
        utility = self.get_utility_vector(agent_index,current_positions)
        formation = self.get_formation_vector(agent_index,current_positions,distribution)

        heading = cw[0]*separation + cw[1]*cohesion + cw[2]*alignment + cw[3]*attraction + cw[4]*utility + cw[5]*formation
        
        return heading
    
    def get_separation_vector(self, agent_index: int, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):

        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_positions[agent_index, :] - current_positions[neighbour, :]

        separation = sum/len(neighbours)
        
        return separation
    
    def get_cohesion_vector(self, agent_index: int, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):
    
        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_positions[neighbour, :] - current_positions[agent_index, :]

        cohesion = sum/len(neighbours)
        
        return cohesion
    
    def get_alignment_vector(self, agent_index: int, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):
        
        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_velocities[neighbour, :] - current_velocities[agent_index, :]

        alignment = sum/len(neighbours)
        
        return alignment
    
    def get_attraction_vector(self, agent_index: int, current_positions: np.ndarray, distribution):

        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += (self.map.get_utility_value(current_positions[neighbour]) - self.map.get_utility_value(current_positions[agent_index]))*(current_positions[neighbour, :] - current_positions[agent_index, :])

        attraction = sum/len(neighbours)
        
        return attraction
    
    def get_utility_vector(self, agent_index: int, current_positions: np.ndarray) -> np.ndarray((2,)):

        utility = self.map.get_gradient(current_positions[agent_index])
        
        return utility
    
    def get_formation_vector(self, agent_index: int, current_positions: np.ndarray, distribution):

        neighbours = np.where(distribution == 1)[0]
        mean_point = np.mean(neighbours)
        rho = np.mean(np.linalg.norm(neighbours-mean_point))
        phis = np.arange(0,2*math.pi, (2*math.pi)/len(neighbours))
        formation_positions = np.zeros((2,len(phis)))
        distance_to_closest_slot = self.sensing_radius
        closest_slot_idx = -1

        for i in range(len(phis)):
            formation_positions[:][i] = np.asarray([rho * np.cos(phis[i]), rho * np.sin(phis[i])])
            distance = np.linalg.norm(current_positions[agent_index] - formation_positions[:][i])
            if distance < distance_to_closest_slot:
                distance_to_closest_slot = distance
                closest_slot_idx = i

        formation = formation_positions[:][closest_slot_idx] - current_positions[agent_index]

        return formation
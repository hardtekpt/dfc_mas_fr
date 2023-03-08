#!/usr/bin/env python3

import numpy as np
from dfc_mas_fr.GradientMap import GradientMap
import math

class Algorithm:
    def __init__(self, agent_id:int, sensing_radius: float, map: GradientMap, control_weights: np.ndarray((6,))):
        self.sensing_radius = sensing_radius
        self.map = map
        self.control_weights = control_weights
        self.agent = agent_id - 1

    def update_movement(self, current_positions: np.ndarray, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):

        cw = self.control_weights
        
        separation = self.get_separation_vector(current_positions,distribution)
        cohesion = self.get_cohesion_vector(current_positions,distribution)
        alignment = self.get_alignment_vector(current_velocities,distribution)
        attraction = self.get_attraction_vector(current_positions,distribution)
        utility = self.get_utility_vector(current_positions)
        formation = self.get_formation_vector(current_positions,distribution)

        heading = cw[0]*separation + cw[1]*cohesion + cw[2]*alignment + cw[3]*attraction + cw[4]*utility + cw[5]*formation

        norm = np.linalg.norm(heading)
        if norm != 0:
            heading /= norm

        new_velocity = current_velocities[self.agent, :] + heading
        norm = np.linalg.norm(new_velocity)
        if norm != 0:
            new_velocity /= norm

        return new_velocity
    
    def get_separation_vector(self, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):

        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_positions[self.agent, :] - current_positions[neighbour, :]

        separation = sum/len(neighbours)
        
        return separation
    
    def get_cohesion_vector(self, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):
    
        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_positions[neighbour, :] - current_positions[self.agent, :]

        cohesion = sum/len(neighbours)
        
        return cohesion
    
    def get_alignment_vector(self, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):
        
        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_velocities[neighbour, :] - current_velocities[self.agent, :]

        alignment = sum/len(neighbours)
        
        return alignment
    
    def get_attraction_vector(self, current_positions: np.ndarray, distribution):

        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += (self.map.get_utility_value(current_positions[neighbour]) - self.map.get_utility_value(current_positions[self.agent]))*(current_positions[neighbour, :] - current_positions[self.agent, :])

        attraction = sum/len(neighbours)
        
        return attraction
    
    def get_utility_vector(self, current_positions: np.ndarray) -> np.ndarray((2,)):

        utility = self.map.get_gradient(current_positions[self.agent])
        
        return utility
    
    def get_formation_vector(self, current_positions: np.ndarray, distribution):

        neighbours = current_positions[np.where(distribution == 1)[0]]
        mean_point = np.asarray([np.mean(neighbours[:,0]), np.mean(neighbours[:,1])])
        #rho = np.mean(np.linalg.norm(neighbours-mean_point, axis=1))
        rho = 0.5
        phis = np.arange(0,2*math.pi, (2*math.pi)/len(neighbours))
        formation_positions = np.zeros((len(phis), 2))
        distance_to_closest_slot = self.sensing_radius
        closest_slot_idx = -1
        neighbours_slot_idx = np.full((len(neighbours),),-1)

        for i in range(len(phis)):
            formation_positions[i] = np.asarray([rho * np.sin(phis[i]), rho * np.cos(phis[i])])
            formation_positions[i] += mean_point
            distance = np.linalg.norm(current_positions[self.agent] - formation_positions[i])
            dist_between_slot_and_neighbours = np.linalg.norm(neighbours - formation_positions[i], axis=1)
            if (distance < distance_to_closest_slot) and (distance == np.min(dist_between_slot_and_neighbours)):
                distance_to_closest_slot = distance
                closest_slot_idx = i

        # When agents compet for slot!!, very slow
        if closest_slot_idx == -1:
            for j in range(len(neighbours)):
                distance_to_closest_slot = 3
                for i in range(len(phis)):
                    distance = np.linalg.norm(neighbours[j] - formation_positions[i])
                    dist_between_slot_and_neighbours = np.linalg.norm(neighbours - formation_positions[i], axis=1)
                    if (distance < distance_to_closest_slot) and (distance == np.min(dist_between_slot_and_neighbours)):
                        distance_to_closest_slot = distance
                        neighbours_slot_idx[j] = i
            for i in range(len(phis)):
                if i not in neighbours_slot_idx:
                    closest_slot_idx = i

        formation = formation_positions[closest_slot_idx] - current_positions[self.agent]

        return formation
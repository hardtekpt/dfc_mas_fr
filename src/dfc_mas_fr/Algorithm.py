#!/usr/bin/env python3

import numpy as np
from dfc_mas_fr.GradientMap import GradientMap
import math

class Algorithm:
    def __init__(self, agent_id:int, sensing_radius:float, map: GradientMap, control_weights: np.ndarray((6,)), collision_params):
        self.sensing_radius = sensing_radius
        self.map = map
        self.control_weights = control_weights
        self.agent = agent_id - 1
        self.collision_params = collision_params

    def normalize(self, vector):

        norm = np.linalg.norm(vector)
        if norm != 0:
            vector /= norm   

        return vector     

    def update_movement(self, current_positions: np.ndarray, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):

        cw = self.control_weights
        
        separation = self.get_separation_vector(current_positions,distribution)
        cohesion = self.get_cohesion_vector(current_positions,distribution)
        alignment = self.get_alignment_vector(current_velocities,distribution)
        attraction = self.get_attraction_vector(current_positions,distribution)
        utility = self.get_utility_vector(current_positions)
        formation = self.get_formation_vector(current_positions,distribution)
        random = self.get_random_vector()

        heading = cw[0]*separation + cw[1]*cohesion + cw[2]*alignment + cw[3]*attraction + cw[4]*utility + cw[5]*formation + cw[6]*random
        heading_norm = self.normalize(heading)

        new_velocity = current_velocities[self.agent, :] + heading

        return new_velocity
    
    def get_separation_vector(self, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):

        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_positions[self.agent, :] - current_positions[neighbour, :]

        separation = sum/len(neighbours)
        separation = self.normalize(separation)
        
        return separation
    
    def get_cohesion_vector(self, current_positions: np.ndarray, distribution) -> np.ndarray((2,)):
    
        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_positions[neighbour, :] - current_positions[self.agent, :]

        cohesion = sum/len(neighbours)
        cohesion = self.normalize(cohesion)
        
        return cohesion
    
    def get_alignment_vector(self, current_velocities: np.ndarray, distribution) -> np.ndarray((2,)):
        
        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += current_velocities[neighbour, :] - current_velocities[self.agent, :]

        alignment = sum/len(neighbours)
        alignment = self.normalize(alignment)
        
        return alignment
    
    def get_attraction_vector(self, current_positions: np.ndarray, distribution):

        neighbours = np.where(distribution == 1)[0]
        sum = 0

        for neighbour in neighbours:
            sum += (self.map.get_utility_value(current_positions[neighbour]) - self.map.get_utility_value(current_positions[self.agent]))*(current_positions[neighbour] - current_positions[self.agent])

        attraction = sum/len(neighbours)
        attraction = self.normalize(attraction)
        
        return attraction
    
    def get_utility_vector(self, current_positions: np.ndarray) -> np.ndarray((2,)):

        utility = self.map.get_gradient(current_positions[self.agent])
        utility = self.normalize(utility)
        
        return utility
    
    def get_formation_vector(self, current_positions: np.ndarray, distribution):

        neighbours = current_positions[np.where(distribution == 1)[0]]

        if len(neighbours) == 1:
            return np.zeros((2,))

        mean_point = np.asarray([np.mean(neighbours[:,0]), np.mean(neighbours[:,1])])
        a = 1.5
        agent_arc_angle = (2*math.pi)/len(neighbours)
        rho = a / (2 * np.sin(agent_arc_angle/2))
        phis = np.arange(0,2*math.pi, agent_arc_angle)
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
                distance_to_closest_slot = self.sensing_radius
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
        formation = self.normalize(formation)

        return formation
    
    def get_random_vector(self):

        angle = np.random.rand() * 2 * math.pi
        random = [np.cos(angle), np.sin(angle)]
        random = self.normalize(random)

        return random
    
    def check_for_agent_collisions(self, heading, current_positions: np.ndarray, distribution):

        neighbours = np.where(distribution == 1)[0]
        heading_norm = self.normalize(heading)
        m = heading[1] / heading[0]
        max_distance_per_time_step = (self.collision_params['max_speed'] * self.collision_params['time_step_size'])
        heading_max = heading_norm * self.collision_params['max_speed']
        expected_agent_position = current_positions[self.agent] + heading_max * self.collision_params['time_step_size']
        earliest = math.inf
        overlapping_distance = 2 * (self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std']) + max_distance_per_time_step
        b = current_positions[self.agent, 1] - m * current_positions[self.agent,0]
        r = overlapping_distance

        for neighbour in neighbours:
            if neighbour == self.agent:
                continue

            distance = np.linalg.norm(expected_agent_position - current_positions[neighbour])

            if distance > overlapping_distance:
                continue
            if distance >= earliest:
                continue

            earliest = distance
            
            h = current_positions[neighbour, 0]
            k = current_positions[neighbour, 1]
            A = 1 + m**2
            B = 2 * m * (b - k) - 2 * h
            C = h**2 - r**2 + (b - k)**2
            roots = np.roots([A, B, C])

            d = np.inf
            for root in roots:
                intersection  = np.asarray([root, m * root + b])
                dd = np.linalg.norm(current_positions[self.agent] - intersection)
                if dd < d:
                    d = dd

            #print(roots, d, max_distance_per_time_step, m)

            # if(d > max_distance_per_time_step):
            #     aux = np.linalg.norm(current_positions[self.agent] - current_positions[neighbour])
            #     if aux < overlapping_distance:
            #         print("agent is already inside neighbour circle")

            heading = self.normalize(heading)
            heading *= d / self.collision_params['time_step_size']

        return heading
    
    def check_for_obstacle_collisions(self, heading, current_positions: np.ndarray, distribution):
    
        heading_norm = self.normalize(heading)
        heading_max = heading_norm * self.collision_params['max_speed']
        expected_agent_position = current_positions[self.agent] + heading_max * self.collision_params['time_step_size']
        overlapping_distance = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std']

        for obs in self.map.map['obstacles']:
            if self.map.check_for_obstacle_collision(expected_agent_position, obs, overlapping_distance):
                return heading * 0

        return heading 

    def check_for_boundaries(self, heading, current_positions: np.ndarray, distribution):

        heading_norm = self.normalize(heading)
        heading_max = heading_norm * self.collision_params['max_speed']
        expected_agent_position = current_positions[self.agent] + heading_max * self.collision_params['time_step_size']
        overlapping_distance = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std']

        if (expected_agent_position[0] + overlapping_distance) > self.map.dimensions[0]:
            return heading * 0

        elif (expected_agent_position[0] - overlapping_distance) < 0:
            return heading * 0

        elif (expected_agent_position[1] + overlapping_distance) > self.map.dimensions[1]:
            return heading * 0
            
        elif (expected_agent_position[1] - overlapping_distance) < 0:
            return heading * 0
        
        return heading

    
    # def check_for_boundaries(self, heading, current_positions: np.ndarray, distribution):
    
    #     heading_norm = self.normalize(heading)
    #     heading_max = heading_norm * self.collision_params['max_speed']
    #     expected_agent_position = current_positions[self.agent] + heading_max * self.collision_params['time_step_size']
    #     overlapping_distance = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std']
    #     distance = math.inf

    #     m = heading_norm[1] / heading_norm[0]
    #     b = current_positions[self.agent, 1] - m * current_positions[self.agent, 0]

    #     intersection = []

    #     if (expected_agent_position[0] + overlapping_distance) > self.map.dimensions[0]:
    #         print(0)
    #         x = self.map.dimensions[0] - overlapping_distance
    #         y = m * x + b
    #         y = current_positions[self.agent,1]
    #         dd = np.linalg.norm(expected_agent_position - [x,y])
    #         if dd < distance:
    #             distance = dd
    #         intersection.append([x,y])

    #     if (expected_agent_position[0] - overlapping_distance) < 0:
    #         print(1)
    #         x = overlapping_distance
    #         y = m * x + b
    #         y = current_positions[self.agent,1]
    #         dd = np.linalg.norm(expected_agent_position - [x,y])
    #         if dd < distance:
    #             distance = dd
    #         intersection.append([x,y])

    #     if (expected_agent_position[1] + overlapping_distance) > self.map.dimensions[1]:
    #         print(2)
    #         y = self.map.dimensions[1] - overlapping_distance
    #         x = (y - b) / m
    #         x = current_positions[self.agent,0]
    #         dd = np.linalg.norm(expected_agent_position - [x,y])
    #         if dd < distance:
    #             distance = dd
    #         intersection.append([x,y])
    
    #     if (expected_agent_position[1] - overlapping_distance) < 0:
    #         print(3)
    #         y = overlapping_distance
    #         x = (y - b) / m
    #         x = current_positions[self.agent,0]
    #         dd = np.linalg.norm(expected_agent_position - [x,y])
    #         if dd < distance:
    #             distance = dd
    #         intersection.append([x,y])

    #     if distance != math.inf:

    #         return heading * 0
    #         sum = 0

    #         for p in intersection:
    #             sum += current_positions[self.agent, :] - p

    #         #heading = sum/len(intersection)

    #         #if np.linalg.norm(heading) > 1:
    #             #heading = self.normalize(heading)
    #     #print(heading, heading_norm, distance != math.inf, current_positions[self.agent, :], intersection)

    #     return heading
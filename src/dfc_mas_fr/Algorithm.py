#!/usr/bin/env python3

import numpy as np
from dfc_mas_fr.GradientMap import GradientMap
import math
import rospy
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

        #heading = cw[0]*separation + cw[1]*cohesion + cw[2]*alignment + cw[3]*attraction + cw[4]*utility + cw[6]*random
        heading = cw[0]*separation + cw[1]*cohesion + cw[2]*alignment + cw[3]*attraction + cw[4]*utility + cw[5]*formation + cw[6]*random
        heading_norm = self.normalize(heading)

        new_velocity = current_velocities[self.agent, :] + heading_norm

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

        for j in range(len(neighbours)):
            distance_to_closest_slot = 10
            for i in range(len(phis)):
                distance = np.linalg.norm(neighbours[j] - formation_positions[i])

                if i not in neighbours_slot_idx:
                    dist_between_slot_and_neighbours = np.linalg.norm(neighbours - formation_positions[i], axis=1)
                else:
                    dist_between_slot_and_neighbours[j] = math.inf

                for k in range(len(phis)):
                    if neighbours_slot_idx[k] != -1:
                        dist_between_slot_and_neighbours[k] = math.inf
                        
                if (distance < distance_to_closest_slot) and (distance == np.min(dist_between_slot_and_neighbours)):
                    distance_to_closest_slot = distance
                    neighbours_slot_idx[j] = i
                    dist_between_slot_and_neighbours[j] = math.inf

        for i in range(len(phis)):
            if i not in neighbours_slot_idx:
                closest_slot_idx = i
        a = 0
        for j in range(len(neighbours)):
            if (current_positions[self.agent,0] == current_positions[j,0]) and (current_positions[self.agent,1] == current_positions[j,1]):
                a = j
        closest_slot_idx = neighbours_slot_idx[a]

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
        max_distance_per_time_step = (self.collision_params['max_speed'] * self.collision_params['time_step_size'])
        agent_radius = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std']
        neighbour_radius = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std'] + max_distance_per_time_step
        a = []
        for neighbour in neighbours:

            Cx = current_positions[self.agent]
            Cy = current_positions[neighbour]
            Vv = heading_norm
            Rx = agent_radius
            Ry = neighbour_radius

            if neighbour == self.agent:
                continue

            if np.linalg.norm(Cx - Cy) <= Rx + Ry:
                continue

            Aa = Vv[0] ** 2 + Vv[1] ** 2
            Bb = 2 * (Cx[0]*Vv[0] - Cy[0]*Vv[0] + Cx[1]*Vv[1] - Cy[1]*Vv[1])
            Cc = Cx[0]**2 + Cy[0]**2 + Cx[1]**2 + Cy[1]**2 - (2 * Cx[1] * Cy[1]) - (2 * Cx[0] * Cy[0]) - (Rx + Ry)**2

            roots = np.roots([Aa, Bb, Cc])

            # if(self.agent == 0):
            #     print(roots, Cx, Cy, Vv, np.linalg.norm(Cx-Cy), rospy.Time.now())

            
            for i in range(len(roots)):
                if isinstance(roots[i], complex) :
                    continue
                if roots[i] < 0:
                   continue
                #    a.append(0)
                a.append(roots[i])

            # if(self.agent == 0):
            #     print(self.agent, a)

            if len(a) == 0:
                continue
            
            #print(heading, heading_norm * (np.min(a) / self.collision_params['time_step_size']))
            heading = heading_norm * np.min(a)
            

        return heading
    
    def check_for_obstacle_collisions(self, heading, current_positions: np.ndarray):
    
        heading_norm = self.normalize(heading)
        heading_max = heading_norm * self.collision_params['max_speed']
        expected_agent_position = current_positions[self.agent] + heading_max * self.collision_params['time_step_size']
        overlapping_distance = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std'] +0.1

        for obs in self.map.map['obstacles']:
            if self.map.check_for_obstacle_collision(expected_agent_position, obs, overlapping_distance):
                return heading * 0

        return heading 

    def check_for_boundaries(self, heading, current_positions: np.ndarray):

        heading_norm = self.normalize(heading)
        heading_max = heading_norm * self.collision_params['max_speed']
        expected_agent_position = current_positions[self.agent] + heading_max * self.collision_params['time_step_size']
        overlapping_distance = self.collision_params['cf_radius'] + 3 * self.collision_params['noise_std'] +0.1

        if (expected_agent_position[0] + overlapping_distance) > self.map.dimensions[0]:
            return heading * 0

        elif (expected_agent_position[0] - overlapping_distance) < 0:
            return heading * 0

        elif (expected_agent_position[1] + overlapping_distance) > self.map.dimensions[1]:
            return heading * 0
            
        elif (expected_agent_position[1] - overlapping_distance) < 0:
            return heading * 0
        
        return heading
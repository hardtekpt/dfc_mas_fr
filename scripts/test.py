#!/usr/bin/env python3

import numpy as np
import math
from matplotlib import pyplot as plt

def abline(slope, intercept):
    """Plot a line from slope and intercept"""
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = intercept + slope * x_vals
    plt.plot(x_vals, y_vals, '--')

def get_formation_vector(agent, current_positions: np.ndarray, distribution):

    plt.rcParams["figure.figsize"] = [5.00, 5.00]
    plt.rcParams["figure.autolayout"] = True
    plt.xlim(-1, 4)
    plt.ylim(-1, 4)
    plt.grid()
    plt.plot(current_positions[:,0], current_positions[:,1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green", linestyle='None')
    
    neighbours = current_positions[np.where(distribution == 1)[0]]
    mean_point = np.asarray([np.mean(neighbours[:,0]), np.mean(neighbours[:,1])])
    rho = np.mean(np.linalg.norm(neighbours-mean_point, axis=1))
    phis = np.arange(0,2*math.pi, (2*math.pi)/len(neighbours))
    formation_positions = np.zeros((len(phis), 2))
    distance_to_closest_slot = 3
    closest_slot_idx = -1
    neighbours_slot_idx = np.full((len(neighbours),),-1)

    print('neighbours', neighbours)
    print('mean', mean_point)
    print('rho', rho)
    print('phis', phis)

    for i in range(len(phis)):
        formation_positions[i] = np.asarray([rho * np.sin(phis[i]), rho * np.cos(phis[i])])
        formation_positions[i] += mean_point

        distance = np.linalg.norm(current_positions[agent] - formation_positions[i])
        dist_between_slot_and_neighbours = np.linalg.norm(neighbours - formation_positions[i], axis=1)

        if (distance < distance_to_closest_slot) and (distance == np.min(dist_between_slot_and_neighbours)):
            distance_to_closest_slot = distance
            closest_slot_idx = i

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


    # if closest_clot_idx is still -1 -> get slot for each neighbour and rm these slots from the formation
    # choose the closest slot from the remaining slots

    plt.plot(formation_positions[:,0], formation_positions[:,1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="red", linestyle='None')

    plt.plot(formation_positions[closest_slot_idx,0], formation_positions[closest_slot_idx,1], marker="x", markersize=15, markeredgecolor="blue", markerfacecolor="blue", linestyle='None')
    plt.plot(current_positions[agent,0], current_positions[agent,1], marker="o", markersize=15, markeredgecolor="blue", markerfacecolor="blue", linestyle='None')
    print(formation_positions)

    print('closest slot', closest_slot_idx, formation_positions[closest_slot_idx])

    
    plt.show()

def normalize( vector):

    norm = np.linalg.norm(vector)
    if norm != 0:
        vector /= norm   

    return vector   

def check_for_agent_collisions(agent, heading, current_positions: np.ndarray, distribution):

    neighbours = np.where(distribution == 1)[0]

    heading = normalize(heading)

    

    max_distance_per_time_step = (1 * 1)
    heading *= 1
    expected_agent_position = current_positions[agent] + heading * 1
    earliest = math.inf
    overlapping_distance = 2 * (0.2 + 3 * 0.05) + max_distance_per_time_step

    print(max_distance_per_time_step, overlapping_distance)
    
    
    m = (expected_agent_position[1] - current_positions[agent,1]) / (expected_agent_position[0] - current_positions[agent,0])
    b = current_positions[agent, 1] - m * current_positions[agent,0]
    r = overlapping_distance
    

    for neighbour in neighbours:
        if neighbour == agent:
            continue

        distance = np.linalg.norm(expected_agent_position - current_positions[neighbour])

        if distance > overlapping_distance:
            continue

        if distance >= earliest:
            continue
        
        h = current_positions[neighbour, 0]
        k = current_positions[neighbour, 1]

        A = 1 + m**2
        B = 2 * m * (b - k) - 2 * h
        C = h**2 - r**2 + (b - k)**2

        p = [A, B, C]
        roots = np.roots(p)

        # x = np.amin(roots)
        # y = m * x + b
        # intersection  = np.asarray([x,y])
        # d = np.linalg.norm(current_positions[agent] - intersection)

        # get the intersection point which is closest to the agent
        d = np.inf
        for root in roots:
            x = root
            y = m * x + b
            intersection  = np.asarray([x,y])
            
            distance = np.linalg.norm(current_positions[agent] - intersection)
            print(intersection, distance)
            if distance < d:
                d = distance

        #d = np.linalg.norm(current_positions[agent] - intersection)

        print(roots, d, max_distance_per_time_step)

        #earliest = distance
        #d = overlapping_distance - earliest
        heading = normalize(heading)
        #heading *= (max_distance_per_time_step - d) / collision_params['time_step_size']
        heading *= d / 1
    
        fig = plt.figure()
        ax = fig.add_subplot()
        abline(m, b)
        plt.plot(current_positions[agent,0], current_positions[agent,1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue", linestyle='None')
        plt.plot(expected_agent_position[0], expected_agent_position[1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="red", linestyle='None')
        plt.plot(intersection[0], intersection[1], marker="x", markersize=10, markeredgecolor="red", markerfacecolor="red", linestyle='None')
        circle1 = plt.Circle(( current_positions[neighbour,0] , current_positions[neighbour,1] ), overlapping_distance )
        circle2 = plt.Circle(( intersection[0] , intersection[1] ), (0.2 + 3 * 0.05) , color='green')
        circle3 = plt.Circle(( current_positions[neighbour,0] , current_positions[neighbour,1] ), overlapping_distance - (0.2 + 3 * 0.05), color='green')
        plt.plot(current_positions[neighbour,0], current_positions[neighbour,1], marker="x", markersize=10, markeredgecolor="blue", markerfacecolor="blue", linestyle='None')
        ax.add_patch(circle1)
        ax.add_patch(circle2)
        ax.add_patch(circle3)
        ax.axis('equal')
        plt.show()

    return heading


if __name__ == '__main__':

    agent = 2
    current_positions = np.asarray([[0.5,1.5],[2.5,2.5],[2,0],[0,1],[1,1],[2,1]])
    dist = np.asarray([1,1,1,1,1,1])

    check_for_agent_collisions(0, np.asarray([0.7,0.7]), current_positions, dist)

    #get_formation_vector(0, current_positions, dist)
    #get_formation_vector(1, current_positions, dist)
    #get_formation_vector(2, current_positions, dist)
    #get_formation_vector(3, current_positions, dist)
    #get_formation_vector(4, current_positions, dist)
    #get_formation_vector(5, current_positions, dist)
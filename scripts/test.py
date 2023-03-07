#!/usr/bin/env python3

import numpy as np
import math
from matplotlib import pyplot as plt

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

if __name__ == '__main__':

    agent = 2
    current_positions = np.asarray([[2,2],[1,0],[2,0],[0,1],[1,1],[2,1]])
    dist = np.asarray([1,1,1,1,1,1])

    #get_formation_vector(0, current_positions, dist)
    #get_formation_vector(1, current_positions, dist)
    #get_formation_vector(2, current_positions, dist)
    get_formation_vector(3, current_positions, dist)
    #get_formation_vector(4, current_positions, dist)
    #get_formation_vector(5, current_positions, dist)
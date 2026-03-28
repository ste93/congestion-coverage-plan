from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
import congestion_coverage_plan.utils.dataset_utils as dataset_utils
import matplotlib.pyplot as plt
from python_tsp.exact import solve_tsp_dynamic_programming
import numpy as np
import itertools


def create_matrix_from_occupancy_map(occupancy_map, level, initial_vertex_id):
    matrix = []

    vertices = occupancy_map.get_vertices()

    for id1 in range(1, len(vertices.keys()) + 1):
        row = []
        for id2 in range(1,len(vertices.keys()) + 1):
            idVertex1 = "vertex" + str(id1)
            idVertex2 = "vertex" + str(id2)
            if idVertex1 == idVertex2:
                row.append(0)
            else:
                edge = occupancy_map.find_edge_from_position(idVertex1, idVertex2)
                if edge is None:
                    edge = occupancy_map.find_edge_from_position(idVertex2, idVertex1)
                if edge is None:
                    row.append(99999999)
                else:

                    traverse_time =occupancy_map.get_edge_traverse_times(edge.get_id())
                    if level == "average":
                        average_traverse_time = 0
                        for level in occupancy_map.get_occupancy_levels():
                            average_traverse_time += traverse_time[level]
                        average_traverse_time = average_traverse_time / len(occupancy_map.get_occupancy_levels())
                        row.append(average_traverse_time)
                    else:
                        row.append(traverse_time[level])

        matrix.append(row)
    return np.array(matrix)


def create_matrix_from_occupancy_map_length_test(occupancy_map, initial_vertex_id):
    matrix = []

    vertices = occupancy_map.get_vertices()
    initial_vertex = occupancy_map.find_vertex_from_id(initial_vertex_id)
    for id1 in range(1, len(vertices.keys()) + 1):
        row = []
        for id2 in range(1,len(vertices.keys()) + 1):
            idVertex1 = "vertex" + str(id1)
            idVertex2 = "vertex" + str(id2)
            if idVertex1 == idVertex2:
                row.append(0)
            else:
                edge = occupancy_map.find_edge_from_position(idVertex1, idVertex2)
                if edge is None:
                    edge = occupancy_map.find_edge_from_position(idVertex2, idVertex1)
                if edge is None:
                    row.append(99999999)
                else:
                    row.append(occupancy_map.find_edge_from_id(edge.get_id()).get_length())

        matrix.append(row)
    return np.array(matrix)


def is_valid_permutation(perm, matrix, starting_number):
    if perm[0] != starting_number:
        return False
    for x in range(0, len(perm) -1):
        if matrix[perm[x]] [perm[x+1]] ==  99999999:
            return False
    return True



def calculate_time(perm, matrix):
    time = 0
    for x in range(0, len(perm) -1):
        time  = time + matrix[perm[x]] [perm[x+1]]
    return time


def solve_tsp(matrix):
    vertices_list = range(0, len(matrix[0]))
    perm_best = None
    time_best = 99999999
    for perm in itertools.permutations(vertices_list):
        if is_valid_permutation(perm, matrix, 0):
            time_current_permutation = calculate_time(perm, matrix)
            if time_current_permutation < time_best:
                time_best = time_current_permutation
                perm_best = perm
    return (time_best, perm_best)



    



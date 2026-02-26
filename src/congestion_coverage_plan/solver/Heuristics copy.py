import math

from congestion_coverage_plan.mdp.MDP import MDP, State
import datetime
from scipy.sparse import csr_array
from scipy.sparse.csgraph import shortest_path
from congestion_coverage_plan.utils import Logger
from scipy.sparse.csgraph import shortest_path, minimum_spanning_tree
import numpy as np
from congestion_coverage_plan.utils import Logger
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import INFINITE_DISTANCE, create_matrix_from_vertices_list_for_mst,  create_matrix_from_vertices_list, solve_with_google_with_data, create_data_model_from_matrix, create_matrix_from_vertices_list_from_shortest_path_matrix_tsp, create_matrix_from_vertices_list_for_mst
import sys


class Heuristics():

    def __init__(self, 
                 occupancy_map, 
                 mdp,
                 heuristic_function="mst_shortest_path",
                 logger = None):
        self.occupancy_map = occupancy_map
        self._mdp = mdp
        # self.logger = logger if logger is not None else Logger()
        self.shortest_paths_matrix = self.calculate_shortest_path_matrix()
        self.minimum_edge_entering_vertices_dict = self.minimum_edge_entering_vertices()
        self.mst_cache = {}  # Cache MST computations
        self.all_vertices_set = set(self.occupancy_map.get_vertices_list())  # Pre-compute for speed
        self.total_vertices = len(self.all_vertices_set)  # For hybrid heuristic
        heuristic_function_name = heuristic_function
        self.heuristic_function = None
        if heuristic_function_name == "teleport":
            self.heuristic_function = self.heuristic_teleport
        elif heuristic_function_name == "mst_shortest_path":
            self.heuristic_function = self.heuristic_mst_shortest_path
        elif heuristic_function_name == "mst":
            self.heuristic_function = self.heuristic_mst
        elif heuristic_function_name == "hybrid_mst":
            self.heuristic_function = self.heuristic_hybrid_mst
        elif heuristic_function_name == "hamiltonian_path":
            self.heuristic_function = self.heuristic_hamiltonian_path
        elif heuristic_function_name == "hamiltonian_path_with_shortest_path":
            self.heuristic_function = self.heuristic_hamiltonian_path_with_shortest_path
        elif heuristic_function_name == "madama_experiments":
            self.heuristic_function = self.heuristic_experiments
        else:
            print("Heuristic function not recognized", heuristic_function_name, "available heuristic functions: teleport, mst_shortest_path, mst, hybrid_mst, hamiltonian_path, hamiltonian_path_with_shortest_path, madama_experiments")
            sys.exit(1)


    def create_matrix_from_vertices_list_for_mst_local(vertices_ids, occupancy_map, initial_vertex_id, shortest_path_matrix=None, value_for_not_existent_edge=INFINITE_DISTANCE):
        matrix = []
        for row_id in range(0, len(vertices_ids)):
            row = []
            vertex_row_id = vertices_ids[row_id]
            for column_id in range(0, len(vertices_ids)):
                vertex_column_id = vertices_ids[column_id]
                # print(row_id, column_id)
                if row_id == column_id:
                    row.append(0)
                else:
                    # print("finding edge from", vertex_row_id, vertex_column_id)
                    edge_length = None
                    if shortest_path_matrix is not None:
                        edge_length = shortest_path_matrix[int(vertex_row_id[6:]) - 1][int(vertex_column_id[6:]) - 1]
                    else:
                        edge = occupancy_map.find_edge_from_position(vertex_row_id, vertex_column_id)
                        if edge is not None:
                            edge_length = edge.get_length()

                    if edge_length is None:
                        row.append(value_for_not_existent_edge)
                    else:
                        row.append(math.floor(edge_length))
            matrix.append(row)
        return matrix



    def create_current_mst_matrix(self, state):
        # create a matrix without the visited vertices
        unvisited = list(self.all_vertices_set - state.get_visited_vertices())
        if state.get_vertex() not in unvisited:
            unvisited.append(state.get_vertex())
        matrix = self.create_matrix_from_vertices_list_for_mst_local(vertices_ids=unvisited, 
                                                          occupancy_map=self.occupancy_map, 
                                                          initial_vertex_id=state.get_vertex(), 
                                                          value_for_not_existent_edge=99999999)

        mst = minimum_spanning_tree(csr_array(matrix))
        return mst.toarray().astype(float)



    

    def heuristic_mst(self, state):
        if self._mdp.solved(state):
            return 0
        
        # Cache key - use frozenset directly (simpler and faster for small sets)
        state_key = (state.to_string())
        if state_key in self.mst_cache:
            print("MST heuristic cache hit for state:", state.to_string(), "cost:", self.mst_cache[state_key])
            return self.mst_cache[state_key]
        
        mst_matrix = self.create_current_mst_matrix(state)
        # Only count each edge once (use upper triangle to avoid double-counting in symmetric matrix)
        cost = np.sum(mst_matrix) / 2  # Since the matrix is symmetric, we can sum all and divide by 2
        if cost is not None:
            self.mst_cache[state_key] = cost
            return cost
        else:
            print("ERRORRRRR: cost is none, this should not happen")
            return None
        # return cost if cost is not None else 9999999

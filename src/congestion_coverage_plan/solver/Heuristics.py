from congestion_coverage_plan.mdp.MDP import MDP, State
import datetime
from scipy.sparse import csr_array
from scipy.sparse.csgraph import shortest_path
from congestion_coverage_plan.utils import Logger
from scipy.sparse.csgraph import shortest_path, minimum_spanning_tree
import numpy as np
from congestion_coverage_plan.utils import Logger
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import create_matrix_from_vertices_list_for_mst,  create_matrix_from_vertices_list, solve_with_google_with_data, create_data_model_from_matrix, create_matrix_from_vertices_list_from_shortest_path_matrix_tsp, create_matrix_from_vertices_list_for_mst
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
        self.heuristic_function = None
        if heuristic_function == "teleport":
            self.heuristic_function = self.heuristic_teleport
        elif heuristic_function == "mst_shortest_path":
            self.heuristic_function = self.heuristic_mst_shortest_path
        elif heuristic_function == "mst":
            self.heuristic_function = self.heuristic_mst
        elif heuristic_function == "hamiltonian_path":
            self.heuristic_function = self.heuristic_hamiltonian_path
        elif heuristic_function == "hamiltonian_path_with_shortest_path":
            self.heuristic_function = self.heuristic_hamiltonian_path_with_shortest_path
        if heuristic_function == "madama_experiments":
            self.heuristic_function = self.heuristic_experiments
        else:
            print("Heuristic function not recognized")
            sys.exit(1)


    ### HEURISTIC HELPERS
    def minimum_edge_entering_vertices(self):
        vertices = self.occupancy_map.get_vertices()
        minimum_edge_entering_vertices = {}
        for vertex_id in vertices.keys():
            for edge in self.occupancy_map.get_edges_from_vertex_with_edge_class(vertex_id):
                if edge.get_length() is not None:
                    if vertex_id not in minimum_edge_entering_vertices:
                        minimum_edge_entering_vertices[vertex_id] = edge.get_length()
                    else:
                        if edge.get_length() < minimum_edge_entering_vertices[vertex_id]:
                            minimum_edge_entering_vertices[vertex_id] = edge.get_length()
        return minimum_edge_entering_vertices


    def create_current_shortest_path_matrix(self, state):
        # create a matrix without the visited vertices
        matrix = create_matrix_from_vertices_list(list(set(self.occupancy_map.get_vertices_list()) - state.get_visited_vertices()) + [state.get_vertex()], self.occupancy_map, state.get_vertex())
        # compute MST
        sp = shortest_path(csr_array(matrix))
        # def calculate_current_shortest_path_matrix(self, state):
        return sp


    def create_current_shortest_path_matrix_for_tsp(self, state):
        # print(self.shortest_paths_matrix)
        matrix = create_matrix_from_vertices_list_from_shortest_path_matrix_tsp(vertices_ids=list(
                                                                                set(self.occupancy_map.get_vertices_list()) -
                                                                                state.get_visited_vertices()) + 
                                                                                [state.get_vertex()], 
                                                                            occupancy_map=self.occupancy_map, 
                                                                            shortest_path_matrix=self.shortest_paths_matrix, 
                                                                            initial_vertex_id=state.get_vertex(), 
                                                                            value_for_not_existent_edge=99999999)
        return matrix


    def create_current_mst_matrix_from_shortest_path(self, state):
        # create a matrix without the visited vertices
        matrix = create_matrix_from_vertices_list_for_mst(vertices_ids=list(set(self.occupancy_map.get_vertices_list()) - state.get_visited_vertices()) + [state.get_vertex()], 
                                                          occupancy_map=self.occupancy_map, 
                                                          initial_vertex_id=state.get_vertex(), 
                                                          shortest_path_matrix=self.shortest_paths_matrix, 
                                                          value_for_not_existent_edge=np.inf)
        mst = minimum_spanning_tree(csr_array(matrix))
        return mst.toarray().astype(float)


    def create_current_mst_matrix(self, state):
        # create a matrix without the visited vertices
        matrix = create_matrix_from_vertices_list_for_mst(vertices_ids=list(set(self.occupancy_map.get_vertices_list()) - state.get_visited_vertices()) + [state.get_vertex()], 
                                                          occupancy_map=self.occupancy_map, 
                                                          initial_vertex_id=state.get_vertex(), 
                                                          value_for_not_existent_edge=99999999)
        # compute MST
        mst = minimum_spanning_tree(csr_array(matrix))
        return mst.toarray().astype(float)


    def calculate_shortest_path(self, vertex1, vertex2):
        vertex1_position = sorted(self.occupancy_map.get_vertices().keys()).index(vertex1)
        vertex2_position = sorted(self.occupancy_map.get_vertices().keys()).index(vertex2)
        return self.shortest_paths_matrix[vertex1_position][vertex2_position]


    def calculate_shortest_path_matrix(self):
        mst_matrix = self.create_map_matrix()
        sp = shortest_path(mst_matrix)
        return sp 


    def create_map_matrix(self):
        vertices = sorted(self.occupancy_map.get_vertices().keys())
        mst_matrix = []
        for vertex in vertices:
            mst_matrix_line = []
            for vertex2 in vertices:
                if vertex == vertex2:
                    mst_matrix_line.append(0)
                elif self.occupancy_map.find_edge_from_position(vertex, vertex2) is not None:
                    edge_id = self.occupancy_map.find_edge_from_position(vertex, vertex2).get_id()
                    mst_matrix_line.append(self.occupancy_map.get_edge_traverse_time(edge_id)['zero'])
                else:
                    mst_matrix_line.append(99999999)
            mst_matrix.append(mst_matrix_line)
        return csr_array(mst_matrix)


    ### HEURISTIC FUNCTIONS
    def heuristic_hamiltonian_path_with_shortest_path(self, state):
        if self._mdp.solved(state):
            return 0
        matrix = self.create_current_shortest_path_matrix_for_tsp(state)
        # print("matrix for hamiltonian path heuristic:", matrix)
        data = create_data_model_from_matrix(matrix)
        cost = solve_with_google_with_data(data)
        if cost is not None:
            return cost
        else:
            print("ERRORRRRR: cost is none, this should not happen")
            return None


    def heuristic_mst(self, state):
        if self._mdp.solved(state):
            return 0
        mst_matrix = self.create_current_mst_matrix(state)
        # check if all the states are connected
        
        # print("matrix for mst heuristic:", mst_matrix)
        cost = np.sum(mst_matrix[mst_matrix != 0])
        if cost is not None:
            return cost
        else:
            print("ERRORRRRR: cost is none, this should not happen")
            return None
        # return cost if cost is not None else 9999999


    def heuristic_mst_shortest_path(self, state):
        if self._mdp.solved(state):
            return 0
        mst_matrix = self.create_current_mst_matrix_from_shortest_path(state)
        # check if all the states are connected
        
        # print("matrix for mst heuristic:", mst_matrix)
        cost = np.sum(mst_matrix[mst_matrix != 0])
        if cost is not None:
            return cost
        else:
            print("ERRORRRRR: cost is none, this should not happen")
            return None
        # return cost if cost is not None else 9999999


    def heuristic_hamiltonian_path(self, state):
        if self._mdp.solved(state):
            return 0
        matrix = create_matrix_from_vertices_list(vertices_ids=list(set(self.occupancy_map.get_vertices_list()) - state.get_visited_vertices()) + [state.get_vertex()], 
                                                  occupancy_map=self.occupancy_map, 
                                                  initial_vertex_id=state.get_vertex(),
                                                  value_for_not_existent_edge=99999999)
                                                #   value_for_not_existent_edge=np.array([np.inf]).astype(int)[0])
        # print("matrix for hamiltonian path heuristic:", matrix)
        data = create_data_model_from_matrix(matrix)
        cost = solve_with_google_with_data(data)
        return cost if cost is not None else 9999999


    def heuristic_teleport(self, state):
        value = 0
        initial_time = datetime.datetime.now()
        if self._mdp.solved(state):
            return 0
        for vertex_id in (self.occupancy_map.get_vertices().keys() - state.get_visited_vertices()):
            value = value + self.minimum_edge_entering_vertices_dict[vertex_id]
        end_time = datetime.datetime.now()
        self.logger.log_time_elapsed("heuristic_teleport::time for calculating heuristic teleport", (end_time - initial_time).total_seconds())
        return value




    def heuristic_experiments(self, state):
        if self._mdp.solved(state):
            return 0
        goal_vertex = self.occupancy_map.find_vertex_from_id(sorted(self.occupancy_map.get_final_goal_vertices())[0])
        current_vertex = self.occupancy_map.find_vertex_from_id(state.get_vertex())
        shortest_path = self.calculate_shortest_path(state.get_vertex(), goal_vertex.get_id())
        remaining_pois_to_explain = len(self.occupancy_map.get_pois_set()) - len(state.get_pois_explained())
        # get current vertex poi
        penalty = 0
        current_vertex_poi = current_vertex.get_poi_number() 
        if current_vertex_poi is not None:
            for poi_number in range(1, current_vertex_poi):
                if poi_number not in state.get_pois_explained():
                    penalty = penalty + 99999
        # increase cost if the pois before are not explained
        
        # check if all the states are connected
        cost = shortest_path + (remaining_pois_to_explain * self._mdp.get_explain_time()) + penalty
        return cost if cost is not None else 9999999

# based on the TopologicalMap class we add information about edge and vertices occupancy, as well as the limits of the edges and vertices in order to consider them as occupied or not.
# the occupancy map is a dictionary with the following structure:
# {
#   'name': 'map_name',
#   'vertex_limits' : [{'id': uuidvertex, 'limit': n_people}, ....],
#   'edge_limits' : [{'id': uuidedge, 'limit': n_people}, ....],
#   'edge_traverse_time': {'uuidedge': {'high': time, 'low': time}, ....}, 
#   'vertex_occupancy': {uuidvertex: high_or_low, ....},
#   'edge_occupancy': {uuidedge: high_or_low, ....},
#   'vertex_expected_occupancy': {time: {uuidvertex: {'high': n_people, 'low': n_people}, ....}, ....},
#   'edge_expected_occupancy': {time: {uuidedge: {'high': n_people, 'low': n_people}, ....}, ....}
# }
# The occupancy map can be saved and loaded to/from a yaml file.
from congestion_coverage_plan.utils import Logger
from tqdm import *
import numpy as np
import math
import yaml
from congestion_coverage_plan.map_utils.TopologicalMap import TopologicalMap
import matplotlib.pyplot as plt
from congestion_coverage_plan.utils.dataset_utils import read_human_traj_data_from_file
import datetime
import asyncio
from multiprocessing.pool import ThreadPool as Pool

class OccupancyMap(TopologicalMap):
    def __init__(self, cliffPredictor, occupancy_levels = ["zero", "one", "two"], people_cost = 10, logger=None):
        self.name = ""
        self.vertex_occupancy = {}
        self.edge_occupancy = {}
        self.vertex_limits = {}
        self.edge_limits = {}
        self.edge_traverse_times = {}
        self.vertex_expected_occupancy = {}
        self.edge_expected_occupancy = {}
        self.cliffPredictor = cliffPredictor
        self.people_trajectories = {}
        self.people_predicted_positions = {}
        self.predicted_positions = None
        self.current_occupancies = {}
        self.occupancy_levels = occupancy_levels
        self.human_traj_data = read_human_traj_data_from_file(self.cliffPredictor.ground_truth_data_file)
        self.already_predicted_times = set()
        self.lock = asyncio.Lock()
        self.people_cost = people_cost
        if logger is not None:
            self.logger = logger
        else:
            self.logger = Logger.Logger(print_time_elapsed=False)
        super().__init__()


    def set_logger(self, logger):
        self.logger = logger


    def set_name(self, name):
        self.name = name


    def get_name(self):
        return self.name


    # load the occupancy map from a yaml file, the file should have the following structure:
    # name: map_name
    # vertex_limits:
    #   - id: uuidvertex
    #     limit: n_people
    # edge_limits:
    #   - id: uuidedge
    #     limit: n_people
    # edge_traverse_times:
    #   uuidedge:
    #     high: time
    #     low: time
    def load_occupancy_map(self, filename):
        print(filename)
        self.load_topological_map(filename.split('.')[0] + "-topological.yaml")
        with open(filename, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self.name = data['name']
            self.occupancy_levels = data['occupancy_levels']
            self.edge_limits = data['edge_limits']
            self.edge_traverse_times = data['edge_traverse_times']


    def plot_tracks_on_map(self):
        self.plot_topological_map(self.cliffPredictor.map_file, self.cliffPredictor.fig_size, self.name)
        
        for person in self.people_trajectories:
            for person_position in self.people_trajectories[person]:
                plt.plot(person_position[1], person_position[2], 'bo')
        plt.show()


    def get_people_collision_cost(self):
        return self.people_cost


    def get_occupancy_levels(self):
        return self.occupancy_levels


    def find_edge_limit(self, edge_id: str):
        if edge_id in self.edge_limits:
            return self.edge_limits[edge_id]
        return 0


    # get the expected occupancy of the edges
    # used when we are in the future and we need to predict the occupancy, weighting the probability of the occupancy
    # the expected occupancy is a dictionary with the following structure:
    # {time: {uuidedge: {'high': n_people, 'low': n_people}, ....}, ....}
    def get_edge_expected_occupancy(self, time, edge_id: str):
        time = math.trunc(time)

        if time not in self.already_predicted_times:
            self._assign_people_to_edge(time, edge_id)
            self._predict_occupancies_for_edge(time, edge_id)
            self.already_predicted_times.add(time)

        if time in self.edge_expected_occupancy:
            if edge_id in self.edge_expected_occupancy[time]:
                return self.edge_expected_occupancy[time][edge_id]
        
        return None


    # get the traverse time of an edge
    def get_edge_traverse_times(self, edge_id: str):
        if edge_id in self.edge_traverse_times:
            return self.edge_traverse_times[edge_id]
        return None


    def get_current_tracks(self):
        # TODO
        return self.people_trajectories


    def calculate_current_occupancies(self, time):
        human_traj_data_by_time = self.human_traj_data.loc[abs(self.human_traj_data['time'] - time) < 1 ]
        self.current_occupancies = {}   
        for index, row in human_traj_data_by_time.iterrows():

            for edge_id in self.edges.keys():
                edge = self.edges[edge_id]
                if edge.is_inside_area(row['x'], row['y']):
                    if edge.get_id() not in self.current_occupancies:
                        self.current_occupancies[edge.get_id()] = 0
                    self.current_occupancies[edge.get_id()] += 1

        return self.current_occupancies


    def predict_occupancies(self, time_now, time_to_predict):
        time_now = time_now
        time_to_predict = math.trunc(time_to_predict)
        self.edge_expected_occupancy = {}
        self.vertex_expected_occupancy = {}
        tracks = self._get_tracks_by_time(time_now)
        self._predict_people_positions(time_now, time_to_predict, tracks)


    def get_current_occupancies(self, time):
        return self.current_occupancies


    # ----- private methods -----


    # used only by predict_occupancies
    def _predict_people_positions(self, time_now, time_to_predict, tracks):
        delta_time = time_to_predict - time_now
        self.people_predicted_positions = {}
        self.people_predicted_positions = self.cliffPredictor.predict_positions(tracks, delta_time)
        return self.people_predicted_positions


    # this is only for simulation purposes
    # used only by predict_occupancies
    def _get_tracks_by_time(self, time):
        human_traj_data_by_time = self.human_traj_data.loc[abs(self.human_traj_data['time'] - time) < 1 ]
        people_ids = list(human_traj_data_by_time.person_id.unique())
        tracks = {}
        self.people_trajectories = {}

        for id in people_ids:
            human_traj_data_by_person_id = self.human_traj_data.loc[self.human_traj_data['person_id'] == id]
            human_traj_array = human_traj_data_by_person_id[["time", "x", "y", "velocity", "motion_angle"]].to_numpy()
            track_before_now = human_traj_array[human_traj_array[:, 0] <= time]
            track_filtered = track_before_now[-(self.cliffPredictor.observed_tracklet_length + 1):]
            if len(track_filtered) >= self.cliffPredictor.observed_tracklet_length:
                tracks[id] = track_filtered

        self.people_trajectories = tracks
        return tracks
    

    def _predict_occupancies_for_edge(self, time: float, edge_id: str):
        time = math.trunc(time)
        if time not in self.edge_expected_occupancy:
            return False
        if edge_id not in self.edge_expected_occupancy[time]:
            return False
        if "poisson_binomial" not in self.edge_expected_occupancy[time][edge_id]:
            self._calculate_poisson_binomial(time)
        for level in self.occupancy_levels:
            self.edge_expected_occupancy[time][edge_id][level] = 0
        for i in range(0, len(self.edge_expected_occupancy[time][edge_id]['poisson_binomial'])):
            for level in self.occupancy_levels:
                if i in range(self.edge_limits[edge_id][level][0], self.edge_limits[edge_id][level][1]):
                    self.edge_expected_occupancy[time][edge_id][level] += self.edge_expected_occupancy[time][edge_id]['poisson_binomial'][i]


    def _poisson_binomial(self, probabilities):
        # probabilities is a list of probabilities
        if not probabilities:
            return np.array([1.0, 0.0])

        P = [1.0-probabilities[0], probabilities[0]] # array of probabilities

        for i in range(1, len(probabilities)):
            new_probabilities = [0.0]*(len(P)+1)

            # fill in the new probabilities
            new_probabilities[0] = P[0]*(1-probabilities[i])
            new_probabilities[len(P)] = P[len(P) - 1]*probabilities[i]

            for j in range(1, len(P)):
                new_probabilities[j] = (P[j-1]*probabilities[i]) + (P[j]*(1-probabilities[i]))
            
            P = new_probabilities
        
        return np.array(P)


    def _calculate_poisson_binomial(self, time):
        for time_probabilities in self.edge_expected_occupancy.keys():
            if time_probabilities == time:
                for edge in self.edge_expected_occupancy[time]:
                    probabilities = self.edge_expected_occupancy[time][edge]['probabilities']
                    poisson_binomial = self._poisson_binomial(probabilities)
                    self.edge_expected_occupancy[time][edge]['poisson_binomial'] = poisson_binomial


    def _assign_people_to_edge(self, time, edge_id):
        time = math.trunc(time)
        if not self.people_predicted_positions:
            return False
        
        if time in self.edge_expected_occupancy:
            if edge_id in self.edge_expected_occupancy[time]:
                if "probabilities" in self.edge_expected_occupancy[time][edge_id]:
                    return True
                
        self.edge_expected_occupancy[time] = {}
        for person_predictions in self.people_predicted_positions:
            self._predict_person_positions(person_predictions, time, edge_id)
        return True


    def _predict_person_positions(self, person_predictions, time, edge_id):
        if person_predictions == []:
            return
        weight_of_prediction = 1 /  len(person_predictions)
        person_edge_occupancy = {}
        for track in person_predictions:
            self._calculate_track_predictions(track, time, person_edge_occupancy, edge_id, weight_of_prediction)

        for edge in person_edge_occupancy:
            self._calculate_edge_probabilities(time, person_edge_occupancy, edge)


    def _calculate_track_predictions(self, track, time, person_edge_occupancy, edge_id, weight_of_prediction):
        track_done = False
        for position in track:
            if abs(position[0] - time) < 1 and not track_done:
                for edge_id_local in self.edges.keys(): 
                    edge = self.edges[edge_id_local]
                    if edge.get_id() == edge_id and edge.is_inside_area(position[1], position[2]):
                        if edge.get_id() not in person_edge_occupancy:
                            person_edge_occupancy[edge.get_id()] = 0
                        person_edge_occupancy[edge.get_id()] += weight_of_prediction
                        track_done = True


    def _calculate_edge_probabilities(self, time, person_edge_occupancy, edge):
        if edge not in self.edge_expected_occupancy[time]:
            self.edge_expected_occupancy[time][edge] = {}
            self.edge_expected_occupancy[time][edge]['probabilities'] = []
        self.edge_expected_occupancy[time][edge]['probabilities'].append(person_edge_occupancy[edge])

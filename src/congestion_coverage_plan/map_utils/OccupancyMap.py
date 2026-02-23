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
from congestion_coverage_plan.map_utils.TopologicalMap import Edge, TopologicalMap
import matplotlib.pyplot as plt
from congestion_coverage_plan.utils.dataset_utils import read_human_traj_data_from_file
import datetime
import asyncio
from congestion_coverage_plan.detections_retriever.DetectionsRetriever import DetectionsRetriever, FakeDetectionsRetriever
import sys


class OccupancyMap(TopologicalMap):
    def __init__(self, cliffPredictor, occupancy_levels = ["zero", "one", "two"], people_cost = 10, logger=None, detections_retriever=None):
        self.name = ""
        np.set_printoptions(suppress=True,formatter={'float_kind':'{:0.2f}'.format}) 
        self.vertex_occupancy = {}
        self.edge_occupancy = {}
        self.vertex_limits = {}
        self.edge_limits = {}
        self.edge_traverse_times = {}
        self.vertex_expected_occupancy = {}
        self._edge_expected_occupancy = {}
        self.cliffPredictor = cliffPredictor
        self.people_predicted_positions = {}
        self.predicted_positions = None
        self.occupancy_levels = occupancy_levels
        # self.human_traj_data = read_human_traj_data_from_file(self.cliffPredictor.ground_truth_data_file)
        self._already_predicted_times = set()
        self.lock = asyncio.Lock()
        self.people_cost = people_cost
        self._current_tracks = None
        self._current_predictions = None
        self._current_occupancies = None
        self._current_time = None

        print("Detections retriever:", detections_retriever)
        if detections_retriever is not None:
            self.detections_retriever = detections_retriever
        else:
            self.detections_retriever = FakeDetectionsRetriever(self.cliffPredictor.ground_truth_data_file)

        if logger is not None:
            self.logger = logger
        else:
            self.logger = Logger.Logger(print_time_elapsed=False)
        super().__init__()


    def set_logger(self, logger):
        self.logger = logger


    def set_time_for_fake_detections_retriever(self, time):
        if isinstance(self.detections_retriever, FakeDetectionsRetriever):
            self.detections_retriever.set_timestamp(time)


    def get_people_collision_cost(self):
        return self.people_cost


    def set_name(self, name: str):
        self.name = name


    def get_name(self):
        return self.name


    def get_people_collision_cost(self):
        return self.people_cost


    def get_occupancy_levels(self):
        return self.occupancy_levels


    def find_edge_limit(self, edge_id: str):
        if edge_id in self.edge_limits:
            return self.edge_limits[edge_id]
        return 0


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
    def load_occupancy_map(self, filename: str):
        print(filename)
        # try catch for loading the topological map and if it fails, print an error message and return false
        try:
            self.load_topological_map(filename.split('.')[0] + "-topological.yaml")
            with open(filename, 'r') as f:
                data = yaml.load(f, Loader=yaml.FullLoader)
                self.name = data['name']
                self.occupancy_levels = data['occupancy_levels']
                self.edge_limits = data['edge_limits']
                self.edge_traverse_times = data['edge_traverse_times']
        except Exception as e:
            print("Error loading occupancy map:", e)
            return False
        return True


    # get the traverse times of an edge
    def get_edge_traverse_times(self, edge_id: str):
        if edge_id in self.edge_traverse_times:
            return self.edge_traverse_times[edge_id]
        return None


    # get the expected occupancy of the edges
    # used when we are in the future and we need to predict the occupancy, weighting the probability of the occupancy
    # the expected occupancy is a dictionary with the following structure:
    # {time: {uuidedge: {'high': n_people, 'low': n_people}, ....}, ....}
    def get_edge_expected_occupancy(self, time, edge_id: str):
        time = math.trunc(time)

        if time not in self._already_predicted_times:
            self._assign_people_predictions_to_edge(time, edge_id)
            self._predict_occupancies_for_edge(time, edge_id)
            self._already_predicted_times.add(time)

        if time in self._edge_expected_occupancy:
            if edge_id in self._edge_expected_occupancy[time]:
                return self._edge_expected_occupancy[time][edge_id]
        
        return None


    def compute_current_tracks(self):
        # get detections returns a dictionary of person_id to list of Detection objects, we need to convert it to a dictionary of person_id to numpy array of positions
        self._current_time = self.detections_retriever.get_current_time()
        # print("---------------- compute_current_tracks, current time:", self._current_time)
        self.human_traj_data = self.detections_retriever.get_detections()

        # print("human_traj_data _to_string:", self.human_traj_data.to_string())
        people_ids = self.human_traj_data.keys()
        tracks = {}
        self._current_tracks = {}
        for id in people_ids:
            # print(f"Person {id} trajectory:")
            human_traj_data_by_person_id = self.human_traj_data[id]
            # for point in human_traj_data_by_person_id:
                # print(f"Person {id} point: time={point.timestamp}, x={point.positionx}, y={point.positiony}, velocity={point.velocity}, motion_angle={point.motion_angle}")
            # convert from list of Detection to numpy array
            datatype = np.dtype([('timestamp', 'f8'), ('x', 'f8'), ('y', 'f8'), ('velocity', 'f8'), ('motion_angle', 'f8')])
            # local_trajectory = np.rec.array([])
            local_trajectory = []
            for detection in human_traj_data_by_person_id:
                local_trajectory.append([float(detection.timestamp), float(detection.positionx), float(detection.positiony), float(detection.velocity), float(detection.motion_angle)])
            # print(f"Person {id} local trajectory (list of lists):", local_trajectory)
            # print(f"Person {id} human_traj_array (list of lists):", human_traj_array)
            # Convert to tuples for structured array creation
            human_traj_struct = np.array([tuple(row) for row in local_trajectory], dtype=datatype)
            # print(f"Person {id} human_traj_struct (numpy structured array):", human_traj_struct)
            # # filter the trajectory to be only before the time
            # print("human_traj_array**:", human_traj_array)
            # print("human_traj_array**:", human_traj_array[0])
            # track_before_now = human_traj_array[human_traj_array[:, 0] <= self._current_time]
            # track_before_now = human_traj_array[human_traj_array[:, 0] <= time]
            sorted_human_traj_array = human_traj_struct[np.argsort(human_traj_struct['timestamp'])]
            track_before_now = sorted_human_traj_array[sorted_human_traj_array['timestamp'] <= self._current_time]
            track_filtered = track_before_now[-(self.cliffPredictor.observed_tracklet_length + 1):]
            if len(track_filtered) >= self.cliffPredictor.observed_tracklet_length:
                tracks[id] = track_filtered
        # print("current tracks @@@@@@@@@@@@@@@:", tracks)
        # here is wrong!!!
        self._current_tracks = tracks


    def calculate_current_occupancies(self):
        self._current_occupancies = {}   
        # use only the positions of the people at the current time, so we filter the trajectories to be only at the current time
        # print(f"DEBUG: _current_tracks has {len(self._current_tracks)} people")
        # print(f"DEBUG: _current_time = {self._current_time}")
        # print(f"DEBUG: edges count = {len(self.edges)}")
        
        for person_id, item in self._current_tracks.items():
            # print(f"DEBUG: Person {person_id} has {len(item)} track points")
            for position in item:
                time_diff = abs(position['timestamp'] - self._current_time)
                # print(f"DEBUG: position timestamp={position['timestamp']}, diff={time_diff:.2f}")
                if time_diff < 1:
                    # print(f"DEBUG: Checking position x={position['x']}, y={position['y']}")
                    for edge_id in self.edges.keys():
                        edge = self.edges[edge_id]
                        if edge.is_inside_area(position['x'], position['y']):
                            # print(f"DEBUG: Position inside edge {edge.get_id()}")
                            if edge.get_id() not in self._current_occupancies:
                                self._current_occupancies[edge.get_id()] = 0
                            self._current_occupancies[edge.get_id()] += 1
        print("################ current occupancies:", self._current_occupancies)


    def get_current_occupancies(self):
        return self._current_occupancies
    

    def plot_tracks_on_map(self):
        self.plot_topological_map(self.cliffPredictor.map_file, self.cliffPredictor.fig_size, self.name)
        
        for person in self._current_tracks:
            for person_position in self._current_tracks[person]:
                plt.plot(person_position['x'], person_position['y'], 'bo')
        plt.show()


    def predict_occupancies(self, time_delta: float):
        self._edge_expected_occupancy = {}
        self.vertex_expected_occupancy = {}
        self._predict_people_positions(time_delta)


    # ----- private methods -----


    # used only by predict_occupancies
    def _predict_people_positions(self, time_delta: float):
        self.people_predicted_positions = {}
        self.people_predicted_positions = self.cliffPredictor.predict_positions(self._current_tracks, time_delta)
        return self.people_predicted_positions


    def _predict_occupancies_for_edge(self, time: float, edge_id: str):
        time = math.trunc(time)
        if time not in self._edge_expected_occupancy:
            return False
        if edge_id not in self._edge_expected_occupancy[time]:
            return False
        if "poisson_binomial" not in self._edge_expected_occupancy[time][edge_id]:
            self._calculate_poisson_binomial(time)
        for level in self.occupancy_levels:
            self._edge_expected_occupancy[time][edge_id][level] = 0
        for i in range(0, len(self._edge_expected_occupancy[time][edge_id]['poisson_binomial'])):
            for level in self.occupancy_levels:
                if i in range(self.edge_limits[edge_id][level][0], self.edge_limits[edge_id][level][1]):
                    self._edge_expected_occupancy[time][edge_id][level] += self._edge_expected_occupancy[time][edge_id]['poisson_binomial'][i]


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
        for time_probabilities in self._edge_expected_occupancy.keys():
            if time_probabilities == time:
                for edge in self._edge_expected_occupancy[time]:
                    probabilities = self._edge_expected_occupancy[time][edge]['probabilities']
                    poisson_binomial = self._poisson_binomial(probabilities)
                    self._edge_expected_occupancy[time][edge]['poisson_binomial'] = poisson_binomial


    def _assign_people_predictions_to_edge(self, time, edge_id):
        time = math.trunc(time)
        if not self.people_predicted_positions:
            return False
        if time in self._edge_expected_occupancy:
            if edge_id in self._edge_expected_occupancy[time]:
                if "probabilities" in self._edge_expected_occupancy[time][edge_id]:
                    return True
        self._edge_expected_occupancy[time] = {}
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
        if edge not in self._edge_expected_occupancy[time]:
            self._edge_expected_occupancy[time][edge] = {}
            self._edge_expected_occupancy[time][edge]['probabilities'] = []
        self._edge_expected_occupancy[time][edge]['probabilities'].append(person_edge_occupancy[edge])

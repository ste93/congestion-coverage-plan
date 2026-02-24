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
        # Caching for predictions
        self._cached_tracks_hash = None
        self._cached_predictions = None
        self._cached_time_delta = None

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
        self.human_traj_data = self.detections_retriever.get_detections()

        people_ids = self.human_traj_data.keys()
        tracks = {}
        self._current_tracks = {}
        
        # Pre-define datatype once
        datatype = np.dtype([('timestamp', 'f8'), ('x', 'f8'), ('y', 'f8'), ('velocity', 'f8'), ('motion_angle', 'f8')])
        min_length = self.cliffPredictor.observed_tracklet_length
        
        for id in people_ids:
            human_traj_data_by_person_id = self.human_traj_data[id]
            
            # Direct numpy array creation without intermediate list
            human_traj_struct = np.array(
                [(float(d.timestamp), float(d.positionx), float(d.positiony), 
                  float(d.velocity), float(d.motion_angle)) 
                 for d in human_traj_data_by_person_id], 
                dtype=datatype
            )
            
            if len(human_traj_struct) == 0:
                continue
                
            # Filter and sort in one pass
            mask = human_traj_struct['timestamp'] <= self._current_time
            track_before_now = human_traj_struct[mask]
            
            if len(track_before_now) == 0:
                continue
            
            # Sort only if needed (check if already sorted)
            if len(track_before_now) > 1 and not np.all(track_before_now['timestamp'][:-1] <= track_before_now['timestamp'][1:]):
                track_before_now = track_before_now[np.argsort(track_before_now['timestamp'])]
            
            track_filtered = track_before_now[-(min_length + 1):]
            if len(track_filtered) >= min_length:
                tracks[id] = track_filtered
        
        self._current_tracks = tracks


    def calculate_current_occupancies(self):
        self._current_occupancies = {}   
        # use only the positions of the people at the current time, so we filter the trajectories to be only at the current time
        
        # Pre-allocate edge list for faster iteration
        edge_ids = list(self.edges.keys())
        
        for person_id, item in self._current_tracks.items():
            # Vectorized time difference calculation
            time_diffs = np.abs(item['timestamp'] - self._current_time)
            recent_mask = time_diffs < 1
            
            if not np.any(recent_mask):
                continue
            
            recent_positions = item[recent_mask]
            
            # Check each recent position against edges
            for position in recent_positions:
                x, y = position['x'], position['y']
                # Check all edges for this position
                for edge_id in edge_ids:
                    edge = self.edges[edge_id]
                    if edge.is_inside_area(x, y):
                        if edge_id not in self._current_occupancies:
                            self._current_occupancies[edge_id] = 0
                        self._current_occupancies[edge_id] += 1
                        break  # Person can only be in one edge at a time
        
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
        import datetime
        t0 = datetime.datetime.now()
        self._edge_expected_occupancy = {}
        self.vertex_expected_occupancy = {}
        t1 = datetime.datetime.now()
        self._predict_people_positions(time_delta)
        t2 = datetime.datetime.now()
        print(f"  - dict clear: {(t1-t0).total_seconds():.3f}s, predict_positions: {(t2-t1).total_seconds():.3f}s")


    # ----- private methods -----


    # used only by predict_occupancies
    def _predict_people_positions(self, time_delta: float):
        # Generate a hash of current tracks for caching
        tracks_hash = self._hash_tracks(self._current_tracks)
        
        # Return cached predictions if tracks and time_delta haven't changed
        if (self._cached_tracks_hash == tracks_hash and 
            self._cached_time_delta == time_delta and 
            self._cached_predictions is not None):
            print("  - Using cached predictions")
            self.people_predicted_positions = self._cached_predictions
            return self.people_predicted_positions
        
        # Compute new predictions
        self.people_predicted_positions = self.cliffPredictor.predict_positions(self._current_tracks, time_delta)
        
        # Cache the results
        self._cached_tracks_hash = tracks_hash
        self._cached_time_delta = time_delta
        self._cached_predictions = self.people_predicted_positions
        
        return self.people_predicted_positions
    
    def _hash_tracks(self, tracks):
        """Generate a hash of tracks for cache comparison."""
        if not tracks:
            return None
        # Create a simple hash based on person IDs and track lengths
        # This is a lightweight check - if tracks structure changes, hash changes
        try:
            return hash((tuple(sorted(tracks.keys())), 
                        tuple(len(tracks[pid]) for pid in sorted(tracks.keys()))))
        except:
            return None


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
        # Don't clear the entire time dict - preserve other edges' predictions
        if time not in self._edge_expected_occupancy:
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
        # Direct edge access instead of iterating through all edges
        edge = self.edges[edge_id]
        
        # Vectorize time check if track is numpy array
        if len(track) > 0:
            if isinstance(track, np.ndarray):
                # Vectorized time filtering
                time_mask = np.abs(track[:, 0] - time) < 1
                valid_positions = track[time_mask]
            else:
                # List of positions - filter by time
                valid_positions = [pos for pos in track if abs(pos[0] - time) < 1]
            
            # Check if any position is inside the edge area
            for position in valid_positions:
                if edge.is_inside_area(position[1], position[2]):
                    # Use setdefault for cleaner initialization
                    person_edge_occupancy[edge_id] = person_edge_occupancy.get(edge_id, 0) + weight_of_prediction
                    return  # Early return once found


    def _calculate_edge_probabilities(self, time, person_edge_occupancy, edge):
        if edge not in self._edge_expected_occupancy[time]:
            self._edge_expected_occupancy[time][edge] = {}
            self._edge_expected_occupancy[time][edge]['probabilities'] = []
        self._edge_expected_occupancy[time][edge]['probabilities'].append(person_edge_occupancy[edge])

from congestion_coverage_plan.detections_retriever.DetectionsRetriever import FakeDetectionsRetriever
from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
import congestion_coverage_plan.utils.dataset_utils as dataset_utils
import matplotlib.pyplot as plt
import csv
from tqdm import tqdm
import numpy as np
from congestion_coverage_plan.cliff_predictor.CliffPredictor import CliffPredictor
from congestion_coverage_plan.cliff_predictor.PredictorCreator import create_atc_cliff_predictor,  create_madama_cliff_predictor
import warnings
import math

class OccupancyMapCreator(OccupancyMap):

    def __init__(self, cliffPredictor, occupancy_map_definition_class, occupancy_levels = ["zero", "one", "two"], people_cost = 10, logger=None):
        super().__init__(cliffPredictor, occupancy_levels, people_cost, logger, FakeDetectionsRetriever(cliffPredictor.ground_truth_data_file))
        self.cliffPredictor = cliffPredictor
        self.human_traj_data = dataset_utils.read_human_traj_data_from_file(cliffPredictor.ground_truth_data_file)
        self.edge_limits = {}
        self.people_cost = people_cost
        self.edge_traverse_times = {}
        self.occupancy_map_definition = occupancy_map_definition_class(None, self, occupancy_levels, "./data/occupancy_maps")


    def _calculate_edge_traverse_time(self, edge_id:str , occupancy_data):
        edge = self.find_edge_from_id(edge_id)
        if edge is None:
            return None
        edge_length = edge.get_length()
        edge_occupancy = 0
        if edge_id in occupancy_data.keys():
            edge_occupancy = occupancy_data[edge_id]
        traverse_time = (edge_length/ 0.3) + edge_occupancy*self.people_cost
        edge_traverse_time = {"num_people" : edge_occupancy, "traverse_time" : traverse_time}
        return edge_traverse_time


    def get_tracks_by_time(self, time):
        # self.detections_retriever.set_timestamp(time)
        self.human_traj_data = self.detections_retriever.get_detections()
        print("human_traj_data:", self.human_traj_data)
        people_ids = self.human_traj_data.keys()
        tracks = {}
        self.people_trajectories = {}

        for id in people_ids:
            human_traj_data_by_person_id = self.human_traj_data[id]
            # convert from list of Detection to numpy array
            datatype = np.dtype([('timestamp', 'f8'), ('x', 'f8'), ('y', 'f8'), ('vx', 'f8'), ('vy', 'f8')])
            # local_trajectory = np.rec.array([])
            local_trajectory = []
            for detection in human_traj_data_by_person_id:
                print("detection:", detection)
                local_trajectory.append([detection.timestamp, detection.positionx, detection.positiony, detection.vx, detection.vy])
            print ("local_trajectory:", local_trajectory)
            human_traj_array = np.array(local_trajectory, dtype=datatype)
            # filter the trajectory to be only before the time
            track_before_now = human_traj_array
            # track_before_now = human_traj_array[human_traj_array[:, 0] <= time]
            track_filtered = track_before_now[-(self.cliffPredictor.observed_tracklet_length + 1):]
            if len(track_filtered) >= self.cliffPredictor.observed_tracklet_length:
                tracks[id] = track_filtered

        self.people_trajectories = tracks
        return tracks



    def _calculate_edges_traverse_times(self, number_of_trials):
        human_traj_data_by_time = self.human_traj_data.time.unique()
        if number_of_trials > len(human_traj_data_by_time):
            number_of_trials = len(human_traj_data_by_time)
        step_length = len(human_traj_data_by_time) // number_of_trials
        traverse_times = {}
        for time_index in tqdm(range(0, len(human_traj_data_by_time), step_length)):
            self.detections_retriever.set_timestamp(human_traj_data_by_time[time_index])
            self.calculate_current_occupancies()
            occupancies = self.get_current_occupancies()
            for edge_key in self.edges:
                if edge_key not in traverse_times.keys():
                    traverse_times[edge_key] = {}
                traverse_time = self._calculate_edge_traverse_time(edge_key, occupancies)
                if traverse_time is not None:
                    for level in self.occupancy_levels:
                        if traverse_time["num_people"] in range(self.edge_limits[edge_key][level][0], self.edge_limits[edge_key][level][1]):
                            if level not in traverse_times[edge_key]:
                                traverse_times[edge_key][level] = []
                            traverse_times[edge_key][level].append(traverse_time["traverse_time"])
        return traverse_times


    def _calculate_edge_traverse_times_with_times(self, times):
        human_traj_data_by_time = self.human_traj_data.time.unique()
        # find indexes for times

        time_indexes = []
        for time in times:
            time_indexes.append(np.where(human_traj_data_by_time == float(time))[0][0])

        traverse_times = {}
        for time_index in tqdm(time_indexes):
            self.calculate_current_occupancies(human_traj_data_by_time[time_index])
            occupancies = self.get_current_occupancies(human_traj_data_by_time[time_index])
            for edge_key in self.edges:
                if edge_key not in traverse_times.keys():
                    traverse_times[edge_key] = {}
                traverse_time = self._calculate_edge_traverse_time(edge_key, occupancies)
                if traverse_time is not None:
                    for level in self.occupancy_levels:
                        if traverse_time["num_people"] in range(self.edge_limits[edge_key][level][0], self.edge_limits[edge_key][level][1]):
                            if level not in traverse_times[edge_key]:
                                traverse_times[edge_key][level] = []
                            traverse_times[edge_key][level].append(traverse_time["traverse_time"])
        return traverse_times


    def calculate_average_edge_occupancy_from_data(self, average_occupancies):
        for edge_id in average_occupancies.keys():
            self.edge_limits[edge_id] = {}
            self.edge_limits[edge_id][self.occupancy_levels[0]] = [0, 0]
            number_of_levels_excluding_zero = len(average_occupancies[edge_id]) - 1
            if number_of_levels_excluding_zero > 0:
                print(edge_id, average_occupancies[edge_id])
                max_occupancy = int(np.max(average_occupancies[edge_id]))
                step_levels = max_occupancy // number_of_levels_excluding_zero
                if max_occupancy > number_of_levels_excluding_zero:
                    step_levels = 1
                previous_level = 1
                for level in self.occupancy_levels[1:]:
                    if level == self.occupancy_levels[-1]:
                        self.edge_limits[edge_id][level] = [previous_level, 99999999]
                    else:
                        self.edge_limits[edge_id][level] = [previous_level, previous_level + step_levels]
                        previous_level += step_levels


    def calculate_average_edge_occupancy_with_times(self, time_list):
        human_traj_data_by_time = self.human_traj_data.time.unique()
        # find indexes for times

        time_indexes = []
        for time in time_list:
            time_indexes.append(np.where(human_traj_data_by_time == float(time))[0][0])

        average_occupancies = {}
        for time_index in tqdm(time_indexes):
            occupancies = self.get_current_occupancies(human_traj_data_by_time[time_index])
            for edge_key in self.edges.keys():
                if edge_key not in average_occupancies.keys():
                    average_occupancies[edge_key] = []
                if edge_key in occupancies and occupancies[edge_key] > 0:
                    average_occupancies[edge_key].append(occupancies[edge_key])

        self.calculate_average_edge_occupancy_from_data(average_occupancies)


    def calculate_average_edge_occupancy(self, number_of_trials):
        human_traj_data_by_time = self.human_traj_data.time.unique()
        if number_of_trials > len(human_traj_data_by_time):
            number_of_trials = len(human_traj_data_by_time)
        step_length = len(human_traj_data_by_time) // number_of_trials
        average_occupancies = {}
        for time_index in tqdm(range(0, len(human_traj_data_by_time), step_length)):
            occupancies = self.get_current_occupancies(human_traj_data_by_time[time_index])
            for edge in self.edges:
                if edge.get_id() not in average_occupancies.keys():
                    average_occupancies[edge.get_id()] = []
                if edge.get_id() in occupancies and occupancies[edge.get_id()] > 0:
                    average_occupancies[edge.get_id()].append(occupancies[edge.get_id()])
        self.calculate_average_edge_occupancy_from_data(average_occupancies)


    def calculate_average_edge_traverse_times_with_time_list(self, time_list):
        traverse_times = self._calculate_edge_traverse_times_with_times(time_list)
        for edge in traverse_times.keys():
            if edge not in self.edge_traverse_times.keys():
                self.edge_traverse_times[edge] = {}
            for level_index in range(0, len(self.occupancy_levels)):
                if level_index == 0:
                    edge_object = self.find_edge_from_id(edge)
                    self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = self._calculate_edge_traverse_time(edge, {})["traverse_time"]
                else:
                    if self.occupancy_levels[level_index] not in traverse_times[edge]:
                        self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = float(self.edge_traverse_times[edge][self.occupancy_levels[level_index - 1]])
                    else:
                        self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = float(np.mean(traverse_times[edge][self.occupancy_levels[level_index]]))


    def calculate_average_edge_traverse_times(self, number_of_trials):
        traverse_times = self._calculate_edges_traverse_times(number_of_trials)

        for edge in traverse_times.keys():
            if edge not in self.edge_traverse_times.keys():
                self.edge_traverse_times[edge] = {}
            for level_index in range(0, len(self.occupancy_levels)):
                if level_index == 0:
                    # print("edge:", edge)
                    edge_object = self.find_edge_from_id(edge)
                    self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = self._calculate_edge_traverse_time(edge, {})["traverse_time"]
                else:
                    if self.occupancy_levels[level_index] not in traverse_times[edge]:
                        # truncate to 3 decimals
                        self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = math.trunc(float(self.edge_traverse_times[edge][self.occupancy_levels[level_index - 1]]) * 1000) / 1000
                    else:
                        self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = math.trunc(float(np.mean(traverse_times[edge][self.occupancy_levels[level_index]])) * 1000) / 1000

    def calculate_average_edge_traverse_times_from_occupancy_levels(self):
        for edge_key in tqdm(self.edges):
            if edge_key not in self.edge_traverse_times.keys():
                self.edge_traverse_times[edge_key] = {}
            for level_index in range(0, len(self.occupancy_levels)):
                average_traverse_time = 0
                max_value = min(self.edge_limits[edge_key][self.occupancy_levels[level_index]][1], 10)
                for occupancy in range(self.edge_limits[edge_key][self.occupancy_levels[level_index]][0],max_value):
                    traverse_time = self._calculate_edge_traverse_time(edge_key, {edge_key: occupancy})
                    average_traverse_time += traverse_time["traverse_time"]
                average_traverse_time = average_traverse_time / (max_value - self.edge_limits[edge_key][self.occupancy_levels[level_index]][0])
                self.edge_traverse_times[edge_key][self.occupancy_levels[level_index]] = average_traverse_time  


    def create_occupancy_map(self):
        self.occupancy_map_definition.vertex_creation_function(self)
        self.occupancy_map_definition.edge_creation_function(self)
        edges = self.get_edges()
        for edge_key in edges:
            self.add_edge_limit(edges[edge_key].get_id(), self.occupancy_levels)
        
        # occupancy_map.calculate_average_edge_traverse_times(num_iterations)
        self.calculate_average_edge_traverse_times_from_occupancy_levels()
        folder = self.occupancy_map_definition.base_folder + self.get_name()
        dataset_utils.create_folder(folder)
        print("Saving occupancy map to folder:", folder)
        filename = folder + '/occupancy_map_' + self.get_name() + "_" + str(len(self.occupancy_levels))+'_levels.yaml'
        self.save_occupancy_map(filename)

if __name__ == "__main__":
    # print(matrix)
    warnings.filterwarnings("ignore")
    # predictor = create_atc_cliff_predictor()
    predictor_madama = create_madama_cliff_predictor()
    # topological_map_creator_function = [create_large_topological_map_atc_corridor, create_medium_large_topological_map_atc_corridor]
    # topological_map_creator_function = [create_large_topological_map_atc_corridor, create_medium_topological_map_atc_corridor, create_small_topological_map_atc_corridor,
    #                                      create_large_topological_map_atc_square, create_medium_topological_map_atc_square]
    # topological_map_creator_function_madama = [create_madama_topological_map_26, create_madama_topological_map_21, create_madama_topological_map_16, create_madama_topological_map_11]
    topological_map_creator_function_madama_doors = [create_madama_topological_map_doors_16]
    # two levels
    occupancy_levels = [
        # (["zero", "one"], {"zero": [0,1], "one": [1,9999999]}),
        # (["zero", "one", "two"], {"zero": [0,1], "one": [1,3], "two": [3,9999999]}),
        # (["zero", "one", "two", "three"], {"zero": [0,1], "one": [1,3], "two": [3,6], "three": [6,9999999]}),
        (["zero", "one", "two", "three", "four"], {"zero": [0,1], "one": [1,3], "two": [3,5], "three": [5,7], "four": [7,9999999]})
        # (["zero", "one", "two", "three", "four", "five"], {"zero": [0,1], "one": [1,3], "two": [3,5], "three": [5,7], "four": [7,9], "five": [9,9999999]}),
        # (["zero", "one", "two", "three", "four", "five", "six"], {"zero": [0,1], "one": [1,3], "two": [3,5], "three": [5,7], "four": [7,9], "five": [9,12], "six": [12,9999999]}),
        # (["zero", "one", "two", "three", "four", "five", "six", "seven"], {"zero": [0,1], "one": [1,2], "two": [2,3], "three": [3,4], "four": [4,5], "five": [5,6], "six": [6,7], "seven": [7,9999999]})
    ]



    for function_name in topological_map_creator_function_madama_doors:
        for levels in occupancy_levels:
            occupancy_map = OccupancyMapCreator(predictor_madama, levels[0])
            occupancy_map.create_occupancy_map()
            # occupancy_map.plot_topological_map(predictor_madama.map_file, predictor_madama.fig_size, occupancy_map.get_name(), show_vertex_names=True)
            # occupancy_map.display_topological_map()

    # for levels in  occupancy_levels:  # just two levels
    #     for vertex_number in [26]:
    #         occupancy_map = OccupancyMap(predictor, levels[0])
    #         function_name_map = globals()[f'create_topological_map_atc_corridor_{vertex_number}']
    #         create_occupancy_map_atc(occupancy_map, levels[1], function_name_map)
            # occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size, occupancy_map.get_name())

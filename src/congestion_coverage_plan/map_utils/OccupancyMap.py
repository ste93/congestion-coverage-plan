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

    def get_people_collision_cost(self):
        return self.people_cost

    def set_name(self, name):
        self.name = name



    def get_name(self):
        return self.name



    # add the traverse time for an edge
    def add_edge_traverse_time(self, edge_id, occupancy_level, time):
        # check if the edge exists
        if self.find_edge_from_id(edge_id) is None:
            return False
        # add the edge traverse time
        if edge_id not in self.edge_traverse_times.keys():
            self.edge_traverse_times[edge_id] = {}
        if occupancy_level in self.edge_traverse_times[edge_id].keys():
            return False
        self.edge_traverse_times[edge_id][occupancy_level] = time
        return True



    def get_occupancy_levels(self):
        return self.occupancy_levels



    # add the limits for vertices and edges
    def add_vertex_limit(self, vertex_id, limit):
        # check if the vertex exists
        if self.find_vertex_from_id(vertex_id) is None:
            return False
        # add the vertex limit
        self.vertex_limits[vertex_id] = limit
        return True



    def add_edge_limit(self, edge_id: str, limit):
        # check if the edge exists
        if self.find_edge_from_id(edge_id) is None:
            return False
        # add the edge limit
        self.edge_limits[edge_id] = limit
        return True




    # find the limits for vertices and edges
    def find_vertex_limit(self, vertex_id):
        if vertex_id in self.vertex_limits:
            return self.vertex_limits[vertex_id]
        return None




    def find_edge_limit(self, edge_id: str):
        if edge_id in self.edge_limits:
            return self.edge_limits[edge_id]
        return 0




    # get the occupancy of the vertices and edges
    def get_edge_expected_occupancy(self, time, edge_id: str):
        time = math.trunc(time)

        if time not in self.already_predicted_times:
            cpu_time_init = datetime.datetime.now()
            self.assign_people_to_edge(time, edge_id)
            cpu_time_end = datetime.datetime.now()
            self.logger.log_time_elapsed("get_edge_expected_occupancy::Time to assign people to edge", (cpu_time_end - cpu_time_init).total_seconds())
            cpu_time_init = datetime.datetime.now()
            self.predict_occupancies_for_edge(time, edge_id)
            self.logger.log_time_elapsed("get_edge_expected_occupancy::Time to predict occupancies for edge", (cpu_time_end - cpu_time_init).total_seconds())
            self.already_predicted_times.add(time)

        if time in self.edge_expected_occupancy:
            if edge_id in self.edge_expected_occupancy[time]:
                return self.edge_expected_occupancy[time][edge_id]
        
        return None



    def get_vertex_expected_occupancy(self, time, vertex_id):
        if time in self.vertex_expected_occupancy:
            if vertex_id in self.vertex_expected_occupancy[time]:
                return self.vertex_expected_occupancy[time][vertex_id]
        return None




    # get the traverse time of an edge
    def get_edge_traverse_time(self, edge_id: str):
        if edge_id in self.edge_traverse_times:
            return self.edge_traverse_times[edge_id]
        return None




    def remove_verted_expected_occupancy(self, time, vertex_id):
        if time in self.vertex_expected_occupancy:
            if vertex_id in self.vertex_expected_occupancy[time]:
                del self.vertex_expected_occupancy[time][vertex_id]




    def remove_edge_expected_occupancy(self, time, edge_id:str):
        if time in self.edge_expected_occupancy:
            if edge_id in self.edge_expected_occupancy[time]:
                del self.edge_expected_occupancy[time][edge_id]




    def _calculate_edge_traverse_time(self, edge_id:str , occupancy_data):
        edge = self.find_edge_from_id(edge_id)
        if edge is None:
            return None
        edge_length = edge.get_length()
        edge_occupancy = 0
        if edge_id in occupancy_data.keys():
            edge_occupancy = occupancy_data[edge_id]
        traverse_time = edge_length + edge_occupancy*self.people_cost
        edge_traverse_time = {"num_people" : edge_occupancy, "traverse_time" : traverse_time}
        return edge_traverse_time




    def _calculate_edges_traverse_times(self, number_of_trials):
        human_traj_data_by_time = self.human_traj_data.time.unique()
        if number_of_trials > len(human_traj_data_by_time):
            number_of_trials = len(human_traj_data_by_time)
        step_length = len(human_traj_data_by_time) // number_of_trials
        traverse_times = {}
        for time_index in tqdm(range(0, len(human_traj_data_by_time), step_length)):
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
                    self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = edge_object.get_length()
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
                    edge_object = self.find_edge_from_id(edge)
                    self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = math.trunc(edge_object.get_length() * 1000) / 1000
                else:
                    if self.occupancy_levels[level_index] not in traverse_times[edge]:
                        # truncate to 3 decimals
                        self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = math.trunc(float(self.edge_traverse_times[edge][self.occupancy_levels[level_index - 1]]) * 1000) / 1000
                    else:
                        self.edge_traverse_times[edge][self.occupancy_levels[level_index]] = math.trunc(float(np.mean(traverse_times[edge][self.occupancy_levels[level_index]])) * 1000) / 1000




    # save and load the occupancy map
    def save_occupancy_map(self, filename):
        self.save_topological_map(filename.split('.')[0] + "-topological." + filename.split('.')[1])
        with open(filename, 'w') as f:
            yaml.Dumper.ignore_aliases = lambda *args : True
            yaml.dump({'name': self.name,
                        'occupancy_levels': self.occupancy_levels,
                        'edge_limits':  self.edge_limits,
                        'edge_traverse_times': self.edge_traverse_times},
                        f,
                        default_flow_style=False)





    def load_occupancy_map(self, filename):
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




    # this is only for simulation purposes
    def get_tracks_by_time(self, time):
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




    def set_edge_limits(self, edge_id, limits_class, limits_values):
        if not self.edge_limits[edge_id]:
            self.edge_limits[edge_id] = {}
        self.edge_limits[edge_id][limits_class] = limits_values




    def get_edge_current_occupancy(self, time, edge):
        with self.lock:
            human_traj_data_by_time = self.human_traj_data.loc[abs(self.human_traj_data['time'] - time) < 1 ]
            if edge.get_id() in self.current_occupancies:
                return self.current_occupancies
            self.current_occupancies[edge.get_id()] = 0
            for index, row in human_traj_data_by_time.iterrows():
                if edge.is_inside_area(row['x'], row['y']):
                    if edge.get_id() not in self.current_occupancies:
                        self.current_occupancies[edge.get_id()] = 0
                    self.current_occupancies[edge.get_id()] += 1

            return self.current_occupancies        




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



    def get_occupancies_by_time(self, time):
        human_traj_data_by_time = self.human_traj_data.loc[abs(self.human_traj_data['time'] - time) < 1 ]
        current_occupancies = {}   
        for index, row in human_traj_data_by_time.iterrows():

            for edge_id in self.edges.keys():
                edge = self.edges[edge_id]
                if edge.is_inside_area(row['x'], row['y']):
                    if edge.get_id() not in current_occupancies:
                        current_occupancies[edge.get_id()] = 0
                    current_occupancies[edge.get_id()] += 1

        return current_occupancies

    def get_current_occupancies(self, time):
        return self.current_occupancies
    

    def plot_tracks_on_map(self):
        self.plot_topological_map(self.cliffPredictor.map_file, self.cliffPredictor.fig_size, self.name)
        
        for person in self.people_trajectories:
            for person_position in self.people_trajectories[person]:
                plt.plot(person_position[1], person_position[2], 'bo')
        plt.show()


    def predict_people_positions(self, time_now, time_to_predict, tracks):
        delta_time = time_to_predict - time_now
        self.people_predicted_positions = {}
        self.people_predicted_positions = self.cliffPredictor.predict_positions(tracks, delta_time)
        return self.people_predicted_positions




    def plot_predicted_positions(self, time):
        self.plot_topological_map()
        for person_prediction in self.people_predicted_positions:
            for trajectory in person_prediction:
                for position in trajectory:
                    if position[0] - time < 0.8:
                        in_area = False
                        for vertex in self.vertices:
                            if vertex.is_inside_area(position[1], position[2]):
                                plt.plot(position[1], position[2], 'ro')
                                in_area = True

                        for edge in self.edges: 
                            if edge.is_inside_area(position[1], position[2]):
                                plt.plot(position[1], position[2], 'ro')
                                in_area = True
                        if not in_area:
                            plt.plot(position[1], position[2], 'go')
        plt.show()




    def predict_occupancies_for_edge(self, time, edge_id):
        time = math.trunc(time)
        if time not in self.edge_expected_occupancy:
            return False
        if edge_id not in self.edge_expected_occupancy[time]:
            return False
        if "poisson_binomial" not in self.edge_expected_occupancy[time][edge_id]:
            self.calculate_poisson_binomial(time)
        for level in self.occupancy_levels:
            self.edge_expected_occupancy[time][edge_id][level] = 0
        for i in range(0, len(self.edge_expected_occupancy[time][edge_id]['poisson_binomial'])):
            for level in self.occupancy_levels:
                if i in range(self.edge_limits[edge_id][level][0], self.edge_limits[edge_id][level][1]):
                    self.edge_expected_occupancy[time][edge_id][level] += self.edge_expected_occupancy[time][edge_id]['poisson_binomial'][i]




    def predict_occupancies(self, time_now, time_to_predict):
        time_now = time_now
        time_to_predict = math.trunc(time_to_predict)
        self.edge_expected_occupancy = {}
        self.vertex_expected_occupancy = {}
        tracks = self.get_tracks_by_time(time_now)
        self.predict_people_positions(time_now, time_to_predict, tracks)




    def get_occupancies(self, time):
        time = math.trunc(time)
        
        if time not in self.vertex_expected_occupancy or time not in self.edge_expected_occupancy:
            return None
        return (self.vertex_expected_occupancy[time], self.edge_expected_occupancy[time])





    def plot_tracked_people(self):
        self.plot_topological_map()
        for person in self.people_trajectories:
            for person_position in self.people_trajectories[person]:
                plt.plot(person_position[1], person_position[2], 'bo')
        plt.show()




    def poisson_binomial(self, probabilities):
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




    def calculate_poisson_binomial(self, time):
        for time_probabilities in self.edge_expected_occupancy.keys():
            if time_probabilities == time:
                for edge in self.edge_expected_occupancy[time]:
                    probabilities = self.edge_expected_occupancy[time][edge]['probabilities']
                    poisson_binomial = self.poisson_binomial(probabilities)
                    self.edge_expected_occupancy[time][edge]['poisson_binomial'] = poisson_binomial





    def assign_people_to_edge(self, time, edge_id):
        time = math.trunc(time)
        if not self.people_predicted_positions:
            return False
        if time in self.edge_expected_occupancy:
            if edge_id in self.edge_expected_occupancy[time]:
                if "probabilities" in self.edge_expected_occupancy[time][edge_id]:
                    return True
        self.edge_expected_occupancy[time] = {}
        cpu_time_init = datetime.datetime.now()
        cpu_time_init = datetime.datetime.now()
        for person_predictions in self.people_predicted_positions:
            self.predict_person_positions(person_predictions, time, edge_id)
        cpu_time_end = datetime.datetime.now()
        self.logger.log_time_elapsed("assign_people_to_edge::Time to predict person positions synchronous", (cpu_time_end - cpu_time_init).total_seconds())


    def predict_person_positions(self, person_predictions, time, edge_id):
        if person_predictions == []:
            return
        weight_of_prediction = 1 /  len(person_predictions)
        person_edge_occupancy = {}
        cpu_time_init = datetime.datetime.now()
        # for track in person_predictions:
        #     self.calculate_track_predictions(track, time, person_edge_occupancy, edge_id, weight_of_prediction)
        cpu_time_init = datetime.datetime.now()
        for track in person_predictions:
            self.calculate_track_predictions(track, time, person_edge_occupancy, edge_id, weight_of_prediction)
        cpu_time_end = datetime.datetime.now()
        self.logger.log_time_elapsed("predict_person_positions::Time to calculate track predictions synchronous", (cpu_time_end - cpu_time_init).total_seconds())

        cpu_time_init = datetime.datetime.now()
        for edge in person_edge_occupancy:
            self.calculate_edge_probabilities(time, person_edge_occupancy, edge)
        cpu_time_end = datetime.datetime.now()
        self.logger.log_time_elapsed("predict_person_positions::Time to calculate edge probabilities", (cpu_time_end - cpu_time_init).total_seconds())

    def calculate_track_predictions(self, track, time, person_edge_occupancy, edge_id, weight_of_prediction):
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

    def calculate_edge_probabilities(self, time, person_edge_occupancy, edge):
        if edge not in self.edge_expected_occupancy[time]:
            self.edge_expected_occupancy[time][edge] = {}
            self.edge_expected_occupancy[time][edge]['probabilities'] = []
        self.edge_expected_occupancy[time][edge]['probabilities'].append(person_edge_occupancy[edge])

    def assign_people_to_areas(self, time):
        time = math.trunc(time)
        if not self.people_predicted_positions:
            return False
        if time in self.edge_expected_occupancy:
            if self.edge_expected_occupancy[time]:
                if "probabilities" in self.edge_expected_occupancy[time]:
                    return True
        self.vertex_expected_occupancy[time] = {}
        self.edge_expected_occupancy[time] = {}
        for person_predictions in self.people_predicted_positions:
            if person_predictions == []:
                continue
            weight_of_prediction = 1 /  len(person_predictions)
            person_edge_occupancy = {}
            for track in person_predictions:
                track_done = False
                for position in track:
                    
                    if abs(position[0] - time) < 1 and not track_done:
                        for edge in self.edges: 
                            if edge.is_inside_area(position[1], position[2]):
                                if edge.get_id() not in person_edge_occupancy:
                                    person_edge_occupancy[edge.get_id()] = 0
                                person_edge_occupancy[edge.get_id()] += weight_of_prediction
                                track_done = True

            for edge in person_edge_occupancy:
                if edge not in self.edge_expected_occupancy[time]:
                    self.edge_expected_occupancy[time][edge] = {}
                    self.edge_expected_occupancy[time][edge]['probabilities'] = []
                self.edge_expected_occupancy[time][edge]['probabilities'].append(person_edge_occupancy[edge])
        return True




    def print_people_in_areas(self, time):
        for vertex in self.vertex_expected_occupancy[time]:
            print("Vertex: ", vertex)
            print("People in area: ", self.vertex_expected_occupancy[time][vertex]['probabilities'])
        
        for edge in self.edge_expected_occupancy[time]:
            print("Edge: ", edge)
            print("People in area: ", self.edge_expected_occupancy[time][edge]['probabilities'])




    def print_poisson_binomial(self, time):
        for vertex in self.vertex_expected_occupancy[time]:
            print("Vertex: ", vertex)
            print("Poisson Binomial: ", self.vertex_expected_occupancy[time][vertex]['poisson_binomial'])
        
        for edge in self.edge_expected_occupancy[time]:
            print("Edge: ", edge)
            print("Poisson Binomial: ", self.edge_expected_occupancy[time][edge]['poisson_binomial'])




    # remove the occupancy of the vertices and edges
    def reset_occupancies(self):
        self.vertex_occupancy = {}
        self.edge_occupancy = {}
        self.vertex_expected_occupancy = {}
        self.edge_expected_occupancy = {}
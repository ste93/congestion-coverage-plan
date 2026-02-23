from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
import math
import congestion_coverage_plan.utils.Logger as Logger 

class State:
    def __init__(self, vertex, time, visited_vertices, pois_explained=None):
        self._vertex = vertex
        self._time = time
        self._visited_vertices = visited_vertices
        self.pois_explained = pois_explained if pois_explained is not None else set()
        self._id = self._calculate_id()

    def __eq__(self, other):
        return self._id == other.get_id()

    def __hash__(self):
        return hash(self._id)
    
    def __str__(self):
        return self.to_string()
    
    def to_string(self):
        return self._id

    def _calculate_id(self):
        visited_vertices_string = ""
        for vertex in sorted(self._visited_vertices):
            visited_vertices_string = visited_vertices_string + " " + str(vertex)
        if self.pois_explained is not None:
            pois_explained_string = ""
            for poi in sorted(self.pois_explained):
                pois_explained_string = pois_explained_string + " " + str(poi)
            return "--current vertex:-- " + str(self._vertex) + " --current time:-- " + str(math.floor(self._time * 100)/100) + " --already visited vertices:-- " + visited_vertices_string + " --pois explained:-- " + pois_explained_string
        return "--current vertex:-- " + str(self._vertex) + " --current time:-- " + str(math.floor(self._time * 100)/100) + " --already visited vertices:-- " + visited_vertices_string + " --pois explained:-- None"

    # getters
    def get_vertex(self):
        return self._vertex  

    def get_time(self):
        return self._time
    
    def get_visited_vertices(self):
        return self._visited_vertices
    
    def set_last_action(self, action):
        self._last_action = action

    def get_last_action(self):
        return self._last_action

    def get_pois_explained(self):
        return self.pois_explained

    def get_id(self):
        return self._id


class Transition:
    '''
    This class represents a transition in the MDP
    it contains the start state, the end state, the action, the cost and the probability of the transition
    '''


    def __init__(self, start, end, action, cost, probability, occupancy_level):
        self._start = start
        self._end = end
        self._action = action
        self._cost = cost
        self._probability = probability
        self._occupancy_level = occupancy_level

    def __eq__(self, other):
        return self._start == other.get_start() and self._end == other.get_end() and self._action == other.get_action() and self._cost == other.get_cost() and self._probability == other.get_probability()
    
    def __hash__(self):
        return hash(self.__str__())
    
    def __str__(self):
        return "--start:--" + str(self._start) + " --end:-- " + str(self._end) + " --action:-- " + str(self._action) + " --cost:-- " + str(self._cost) + " --probability:-- " + str(self._probability) + " --occupancy level:-- " + str(self._occupancy_level)
    # create getters for the class
    
    def to_string(self):
        return "--start:--" + str(self._start) + " --end:-- " + str(self._end) + " --action:-- " + str(self._action) + " --cost:-- " + str(self._cost) + " --probability:-- " + str(self._probability) + " --occupancy level:-- " + str(self._occupancy_level)

    def get_start(self):
        return self._start
    
    def get_end(self):
        return self._end
    
    def get_action(self):
        return self._action
    
    def get_cost(self):
        return self._cost
    
    def get_probability(self):
        return self._probability
    
    def get_occupancy_level(self):
        return self._occupancy_level


class MDP:
    def __init__(self, occupancy_map, time_for_occupancies , time_start, wait_time, explain_time, logger=None, do_museum_trials=False):
        self.occupancy_map = occupancy_map
        self.time_start = time_start
        self.time_for_occupancies = time_for_occupancies
        self._wait_time = wait_time
        self._explain_time = explain_time
        if logger is not None:
            self.logger = logger
        else:
            self.logger = Logger.Logger(print_time_elapsed=False)

        self.solved = None
        self.compute_next_state = None
        self.get_possible_actions = None
        if do_museum_trials:
            self.solved = self.solved_museum
            self.compute_next_state = self.compute_next_state_museum
            self.get_possible_actions = self.get_possible_actions_museum
        else:            
            self.solved = self.solved_coverage
            self.compute_next_state = self.compute_next_state_coverage
            self.get_possible_actions = self.get_possible_actions_coverage


    def get_explain_time(self):
        return self._explain_time


    def compute_transition(self, state,  edge, occupancy_level, transitions_list):
        # print ("compute_transition", state, edge, occupancy_level, transitions_list)
        # cpu_time_init = datetime.datetime.now()
        if edge is None:
            print("@@@@@@@@@@@@@@@@Edge not found", state.to_string(), " +++++ ")
            return
        transition_probability = self.calculate_transition_probability(edge,self.time_for_occupancies + state.get_time() - self.time_start, occupancy_level)
        # cpu_time_end = datetime.datetime.now()
        # cpu_time = (cpu_time_end - cpu_time_init).total_seconds()
        # print("compute_transition::CPU time for calculate_transition_probability: ", cpu_time)
        # print ("transition_probability", transition_probability)
        if transition_probability < 0.000001:
            return
        # cpu_time_init = datetime.datetime.now()
        transition_cost = self.calculate_transition_cost(edge,self.time_for_occupancies + state.get_time() - self.time_start , occupancy_level)
        # if edge.get_end() in state.get_visited_vertices():
            # print("Edge already visited", edge.get_end(), "in", state.get_visited_vertices())
            # transition_cost = transition_cost * 2
        # cpu_time_end = datetime.datetime.now()
        # cpu_time = (cpu_time_end - cpu_time_init).total_seconds()
        # print("compute_transition::CPU time for calculate_transition_cost: ", cpu_time)
        transitions_list.append(Transition(start=edge.get_start(), 
                          end=edge.get_end(), 
                          action=edge.get_end(),
                          cost=transition_cost,
                          probability=transition_probability,
                          occupancy_level=occupancy_level))



    def calculate_transition_probability(self, edge, time, occupancy_level):

        edge_limits = self.occupancy_map.find_edge_limit(edge.get_id())[occupancy_level]
        if time - self.time_for_occupancies < 1:
            occupancies = self.occupancy_map.get_current_occupancies()
            edge_occupancy = 0
            if edge.get_id() not in occupancies.keys():
                if occupancy_level == self.occupancy_map.get_occupancy_levels()[0]:
                    return 1
                else:
                    return 0
            edge_occupancy = occupancies[edge.get_id()]
            if edge_occupancy in range(edge_limits[0], edge_limits[1]):
                return 1
            else:
                return 0
        else:
            # in this case we are in the future and we need to predict the occupancy, weighting the probability of the occupancy
            # cpu_time_init = datetime.datetime.now()
            occupancies = self.occupancy_map.get_edge_expected_occupancy(time,  edge.get_id())
            # cpu_time_end = datetime.datetime.now()
            # cpu_time = (cpu_time_end - cpu_time_init).total_seconds()
            # print("calculate_transition_probability::CPU time for get_edge_expected_occupancy: ", cpu_time)
            if (occupancies):
                # if I have predicted occupancies
                # cpu_time_init = datetime.datetime.now()
                sum_poisson_binomial = 0
                for x in range(edge_limits[0], min(edge_limits[1], len(occupancies["poisson_binomial"]))):
                    sum_poisson_binomial = sum_poisson_binomial + occupancies["poisson_binomial"][x]
                # better implementation of the poisson binomial
                # cpu_time_end = datetime.datetime.now()
                # cpu_time = (cpu_time_end - cpu_time_init).total_seconds()
                # print("calculate_transition_probability::CPU time for poisson_binomial: ", cpu_time)
                return sum_poisson_binomial
            # if I have not predicted occupancies I will return the zero occupancy
            else:
                # if I have not predicted occupancies all except the low occupancy will be zero probability
                if occupancy_level == self.occupancy_map.get_occupancy_levels()[0]:
                    return 1
                else:
                    return 0



    def calculate_transition_cost(self, edge, time, occupancy_level):
        return math.trunc(self.occupancy_map.get_edge_traverse_times(edge.get_id())[occupancy_level] * 100) / 100.0


    def get_possible_transitions_from_action(self, state, action, time_bound):
        #returns a set of transitions
        if state.get_time() > time_bound or self.solved(state):
            return []
        # print(action, "action")
        if action == "wait":
            # start, end, action, cost, probability, occupancy_level
            return [Transition(state.get_vertex(), state.get_vertex(), "wait", self._wait_time, 1, "none")]
        elif action == "explain":
            return [Transition(state.get_vertex(), state.get_vertex(), "explain", self._explain_time, 1, "none")]
        elif action == "end":
            return [Transition(state.get_vertex(), state.get_vertex(), "end", 99999999, 1, "none")]
        else:
            # print("action:", action, "state", state.to_string())
            transitions = []
            pairs = []

            edge = self.occupancy_map.find_edge_from_position(state.get_vertex(), action)
            if edge is None:
                print("Edge not found", state.to_string(), " +++++ " , action)
            for occupancy_level in self.occupancy_map.get_occupancy_levels():
                pairs.append((edge, occupancy_level))

            for item in pairs:
                self.compute_transition(State(vertex=state.get_vertex(), 
                                              time=state.get_time(), 
                                              visited_vertices=state.get_visited_vertices(), 
                                              pois_explained=state.get_pois_explained()), 
                                        item[0], item[1], transitions)
            return transitions


    def get_possible_actions_museum(self, state):
        # actions = list(set(self.occupancy_map.get_edges_from_vertex(state.get_vertex()).copy() ) - state.get_visited_vertices()) + ["wait"]
        actions = list(set(self.occupancy_map.get_edges_from_vertex(state.get_vertex()).copy()))
        vertex = self.occupancy_map.find_vertex_from_id(state.get_vertex())
        if vertex.get_poi_number() is not None and (vertex.get_poi_number() not in state.get_pois_explained()):
            actions.append("explain")
        actions.append("end")
        # actions = list(set(self.occupancy_map.get_edges_from_vertex(state.get_vertex()).copy()))
        return actions


    def get_possible_actions_coverage(self, state):
        # actions = list(set(self.occupancy_map.get_edges_from_vertex(state.get_vertex()).copy() ) - state.get_visited_vertices()) + ["wait"]
        actions = list(set(self.occupancy_map.get_edges_from_vertex(state.get_vertex()).copy() ))
        # if state.get_last_action() is not None and state.get_last_action() != "wait":
        actions.append("wait")
        # actions = list(set(self.occupancy_map.get_edges_from_vertex(state.get_vertex()).copy()))
        return actions


    def compute_next_state_coverage(self, state, transition):
        #returns a single next state
        visited_vertices = state.get_visited_vertices() | set([transition.get_end()])

        return State(vertex=transition.get_end(), 
                     time=state.get_time() + transition.get_cost(), 
                     visited_vertices=visited_vertices, 
                     pois_explained=state.get_pois_explained())



    def compute_next_state_museum(self, state, transition):
        #returns a single next state
        if transition.get_action() == "explain":
            current_vertex = self.occupancy_map.find_vertex_from_id(state.get_vertex())
            pois_explained = state.get_pois_explained().union(set([current_vertex.get_poi_number()]))
            
            print("Explaining POI:", current_vertex.get_poi_number(), "Total explained:", pois_explained)
            return State(vertex=state.get_vertex(), 
                         time=state.get_time() + transition.get_cost(), 
                         visited_vertices=state.get_visited_vertices(), 
                         pois_explained=pois_explained)
        visited_vertices = state.get_visited_vertices() | set([transition.get_end()])
        return State(vertex=transition.get_end(), 
                     time=state.get_time() + transition.get_cost(), 
                     visited_vertices=visited_vertices, 
                     pois_explained=state.get_pois_explained())



    def solved_museum(self, state):
        # difference = len(self.occupancy_map.get_vertices().keys()) - len(state.get_visited_vertices())
        
        if state.get_vertex() not in self.occupancy_map.get_final_goal_vertices():
            return False
        # difference = len(self.occupancy_map.get_pois_set()) - len(state.get_pois_explained())
        for poi in self.occupancy_map.get_pois_set():
            if poi not in state.get_pois_explained():
                return False

        # solved = difference == 0 
        return True


    def solved_coverage(self, state):
        difference = len(self.occupancy_map.get_vertices().keys()) - len(state.get_visited_vertices())
        solved = difference == 0
        # print("Checking if solved, state:", state.to_string(), "difference:", difference, "solved:", solved)

        return solved

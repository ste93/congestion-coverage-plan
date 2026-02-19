from congestion_coverage_plan.mdp.MDP import MDP, State
import datetime
from scipy.sparse import csr_array
from scipy.sparse.csgraph import shortest_path, minimum_spanning_tree
import numpy as np
from congestion_coverage_plan.utils import Logger
from congestion_coverage_plan.solver.Heuristics import Heuristics
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import create_matrix_from_vertices_list_for_mst,  create_matrix_from_vertices_list, solve_with_google_with_data, create_data_model_from_matrix, create_matrix_from_vertices_list_from_shortest_path_matrix_tsp, create_matrix_from_vertices_list_for_mst
import sys
class LrtdpTvmaAlgorithm():

    def __init__(self, 
                 occupancy_map, 
                 initial_state_name,
                 convergence_threshold, 
                 planning_time_bound, 
                 solution_time_bound, 
                 time_for_occupancies, 
                 time_start , 
                 wait_time, 
                 heuristic_function, 
                 vinitState=None, 
                 logger=None):
        self.occupancy_map = occupancy_map

        self.mdp = MDP(occupancy_map=occupancy_map, 
                       time_for_occupancies=time_for_occupancies, 
                       time_start=time_start, 
                       wait_time=wait_time)
        self._wait_time = wait_time
        self.initial_time = time_for_occupancies
        self.time_for_occupancies = time_for_occupancies
        if vinitState is not None:
            self.vinitState = vinitState
            self.initial_time = time_for_occupancies
        else:
            self.vinitState = State(initial_state_name, 
                                   0,
                                    set([initial_state_name]), None)
        self.vinitStateName = initial_state_name
        self.planning_time_bound = planning_time_bound
        self.solution_time_bound = solution_time_bound
        self.convergenceThresholdGlobal = convergence_threshold
        self.policy = {}
        self.valueFunction = {}
        self.action_costs = {}
        self.solved_set = set()
        # self.shortest_paths_matrix = self.calculate_shortest_path_matrix()
        # self.minimum_edge_entering_vertices_dict = self.minimum_edge_entering_vertices()
        heuristics = Heuristics(occupancy_map=self.occupancy_map,
                                mdp=self.mdp,
                                heuristic_function=heuristic_function,
                                logger=logger)
        if logger is not None:
            self.logger = logger
        else:
            self.logger = Logger.Logger(print_time_elapsed=False)

        self.heuristic_function = heuristics.heuristic_function
        self.heuristic_backup = {}
        
    ### HELPERS
    def get_policy(self):
        return self.policy


    ### Q VALUES
    def calculate_Q(self, state, action):
        if self.goal(state):
            return 0
        
        current_action_cost = 0
        future_actions_cost = 0

        possible_transitions = self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound)

        for transition in possible_transitions:

            if transition.get_probability() == 0:
                continue
            # if transition.get_occupancy_level() != "none" and transition.get_occupancy_level() != "zero":
            #     print("calculate_Q::transition with occupancy level:", transition.to_string())
            # print("calculate_Q::transition with occupancy level:", transition.to_string())
            local_current_action_cost = 0
            local_current_action_cost = transition.get_cost() * transition.get_probability()
            current_action_cost = current_action_cost + local_current_action_cost

            next_state = self.mdp.compute_next_state(state, transition)
            local_future_actions_cost = self.get_value(next_state) * transition.get_probability()
            
            future_actions_cost = future_actions_cost + local_future_actions_cost
        # self.qvalues[state.to_string() + action] = current_action_cost + future_actions_cost
        cost = current_action_cost + future_actions_cost
        # if cost  <= 0:
            # print("errorrrrr", cost, state.to_string(), action, current_action_cost, future_actions_cost)
            # print(len(possible_transitions))
            # for transition in possible_transitions:
            #     print(transition.get_cost(), transition.get_probability())
        # if state.get_time() == 0:
        #     print("Q value for state:", state.to_string(), "action:", action, "current action cost:", current_action_cost, "future actions cost:", future_actions_cost, "is:", cost)

        return cost


    def calculate_argmin_Q(self, state):
        qvalues = []
        state_internal = State(state.get_vertex(), state.get_time(), state.get_visited_vertices().copy(), state.get_last_action())
        time_initial = datetime.datetime.now()
        possible_actions = self.mdp.get_possible_actions(state_internal)
        if not possible_actions:
            print("NO POSSIBLE ACTIONS???")
            return (0, state_internal, "")
        time_final = datetime.datetime.now()
        self.logger.log_time_elapsed("calculate_argmin_Q::time for getting possible actions", (time_final - time_initial).total_seconds())

        # actions_sorted = list(possible_actions)
        # actions_sorted.sort()
        time_initial = datetime.datetime.now()
        for action in possible_actions:
            qvalues.append((self.calculate_Q(state_internal, action), state_internal, action))
            # if state.get_time() == 0:
                # print("Q value for state:", state_internal.to_string(), "action:", action, "is:", qvalues[-1][0])
        self.logger.log_time_elapsed("calculate_argmin_Q::time for calculating Q values", (time_final - time_initial).total_seconds())

        time_initial = datetime.datetime.now()
        min = None
        # min = np.min(qvalues, key=lambda x: x[0])  # Find the minimum Q value
        for qvalue in qvalues:
            if min is None:
                min = qvalue
            else:
                if qvalue[0] < min[0]:
                    min = qvalue
        time_final = datetime.datetime.now()
        self.logger.log_time_elapsed("calculate_argmin_Q::time for finding minimum Q value", (time_final - time_initial).total_seconds())
        # if min[0] <= 0:
        #     print("goal")
        return (min[0], min[1], min[2]) # this contains the value, state and action with the minimum Q value


    ### STATE FUNCTIONS
    def update(self, state):
        action = self.greedy_action(state)
        self.valueFunction[state.to_string()] = action[0]  # this is the value of the state
        return True


    def greedy_action(self, state):
        return self.calculate_argmin_Q(state)


    def residual(self, state):
        # print("Residual for state:", state.to_string())

        action = self.greedy_action(state)
        residual = abs(self.get_value(state) - action[0])
        # if residual > 0.01:
        #     print("Residual for state:", state.to_string(), "======", residual, "GOAL?  ", self.goal(state))
        # if residual < 0.0001:
        #     print("**************************", action[0], action[1].to_string(), action[2])
        #     print(state.to_string())
        #     print()
        return residual


    def solved(self, state):
        return state.to_string() in self.solved_set


    def get_value(self, state):
        if state.to_string() in self.valueFunction:
            return self.valueFunction[state.to_string()]
        if state.to_string() in self.heuristic_backup:
            return self.heuristic_backup[state.to_string()]
        heuristic_value = self.heuristic_function(state)
        self.heuristic_backup[state.to_string()] = heuristic_value
        return heuristic_value


    def goal(self, state):
        is_goal = self.mdp.solved(state)
        return is_goal


    def check_solved(self, state, thetaparameter):
        # print("Checking if state is solved: ", state.to_string())
        solved_condition = True
        open = []
        closed = []
        open.append(state)
        while open != []:
            state = open.pop()
            closed.append(state)
            if self.residual(state) > thetaparameter: # or state.get_time() > self.solution_time_bound:
                solved_condition = False
                continue

            action = self.greedy_action(state)[2] # get the greedy action for the state            
            for transition in self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound):
                next_state = self.mdp.compute_next_state(state, transition)
                if not (next_state in open or next_state in closed) and not self.solved(next_state): # and not self.goal(state): # and next_state.get_time() <= self.solution_time_bound: # and not self.goal(next_state):
                    open.append(next_state)
        if solved_condition:
            for state in closed:
                self.solved_set.add(state.to_string())
        else:
            while closed:
                state = closed.pop()
                self.update(state)
        return solved_condition


    def calculate_most_probable_transition(self, state, action):
        most_probable_transitions = []

        for transition in self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound):
            if transition.get_probability() > 0:
                if most_probable_transitions == []:
                    most_probable_transitions.append(transition)
                else:
                    if transition.get_probability() < most_probable_transitions[0].get_probability():
                        most_probable_transitions = []
                        most_probable_transitions.append(transition)
                    elif transition.get_probability() == most_probable_transitions[0].get_probability():
                        most_probable_transitions.append(transition)
        most_probable_transition_to_return = None

        if len(most_probable_transitions) > 1:
            # get the one with the lowest cost
            min_cost = None
            for transition in most_probable_transitions:
                if min_cost is None:
                    min_cost = transition.get_cost()
                    most_probable_transition_to_return = transition
                else:
                    if transition.get_cost() < min_cost:
                        min_cost = transition.get_cost()
                        most_probable_transition_to_return = transition
        else:
            most_probable_transition_to_return = most_probable_transitions[0]
        return most_probable_transition_to_return


    def lrtdp_tvma(self):
        number_of_trials = 0
        self.occupancy_map.predict_occupancies(self.time_for_occupancies, self.time_for_occupancies + self.solution_time_bound)
        self.occupancy_map.calculate_current_occupancies(self.time_for_occupancies)
        initial_current_time = datetime.datetime.now()
        # for edge_id in self.occupancy_map.get_edges().keys():
        #     # print the current occupancies for each edge
        # occ = self.occupancy_map.get_current_occupancies(edge_id)
        # if occ is not None:
        #     print("Edge: ", edge_id, "occupancy levels: ", occ)
            # print("Edge: ", edge_id, "occupancy levels: ", occ)
        print("LRTDP TVMA started at: ", initial_current_time, "convergence threshold:", self.convergenceThresholdGlobal, "wait_time:", self._wait_time, "planner time bound:", self.solution_time_bound, "real time bound:", self.planning_time_bound, "initial time for occupancies:", self.time_for_occupancies)
        average_trial_time = 0
        old_policy = None
        old_time = None
        while (not self.solved(self.vinitState)) and ((datetime.datetime.now() - initial_current_time)) < datetime.timedelta(seconds = self.planning_time_bound):
            time_init_trial = datetime.datetime.now()
            # print("Trial number: ", number_of_trials)
            self.lrtdp_tvma_trial(self.vinitState, self.convergenceThresholdGlobal, self.solution_time_bound)
            # print(self.valueFunction)
            # print(self.policy)
            # for item in self.policy.keys():
            #     print("***state***", item, "***qvalue***", self.policy[item][0], "***action***", self.policy[item][2])
            time_final_trial = datetime.datetime.now()
            self.logger.log_time_elapsed("trial time", (time_final_trial - time_init_trial).total_seconds())
            # print("Trial ", number_of_trials, " time: ", (time_final_trial - time_init_trial).total_seconds())
            number_of_trials += 1
            average_trial_time = (average_trial_time * (number_of_trials - 1) + (time_final_trial - time_init_trial).total_seconds()) / number_of_trials
            if number_of_trials % 50 == 0:
                print("Average trial time after " + str(number_of_trials) + " trials: ", average_trial_time)
                print(len(self.policy), "states in policy")
                # print("Current policy: ", self.policy)
                print(len(self.valueFunction), "states in value function")
            # if old_policy == self.policy[self.vinitState.to_string()][2] and old_time == self.policy[self.vinitState.to_string()][0]:
            #     print("Policy has not changed.", old_policy, "**", old_time)
            # else:
            #     print("Policy has changed.", old_policy, "**", old_time, "->", self.policy[self.vinitState.to_string()][2], "**", self.policy[self.vinitState.to_string()][0])
            old_policy = self.policy[self.vinitState.to_string()][2] if self.vinitState.to_string() in self.policy else None
            old_time = self.policy[self.vinitState.to_string()][0] if self.vinitState.to_string() in self.policy else None
        print(str(number_of_trials) + " trials")
        print(len(self.policy), "states in policy")
        print(len(self.valueFunction), "states in value function")
        return self.solved(self.vinitState)


    def lrtdp_tvma_trial(self, vinitStateParameter, thetaparameter, solution_time_bound):
            # print("trial started")
            visited = [] # this is a stack
            state = vinitStateParameter
            while not self.solved(state):
                # print("checking state:", state.to_string())

                visited.append(state)
                self.update(state)
                self.policy[state.to_string()] = self.calculate_argmin_Q(state)
                if self.goal(state) or (state.get_time() > solution_time_bound):
                    ######## should there be here a bellamn backup?
                    break
                # perform bellman backup and update policy
                time_initial = datetime.datetime.now()
                time_final = datetime.datetime.now()
                self.logger.log_time_elapsed("lrtdp_tvma_trial::time for argmin", (time_final - time_initial).total_seconds())
                # print("state: ", state.to_string())
                # print("action: ", self.policy[state.to_string()][2])
                time_initial = datetime.datetime.now()
                action = self.policy[state.to_string()][2]
                transitions = self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound)
                if not transitions:
                    print("lrtdp_tvma_trial::No transitions found for state: ", state.to_string())
                    break
                time_final = datetime.datetime.now()
                self.logger.log_time_elapsed("lrtdp_tvma_trial::time for transitions", (time_final - time_initial).total_seconds())
                # for t in transitions:
                #     print("transition: ", t.to_string())
                time_initial = datetime.datetime.now()
                transition_selected = np.random.choice(transitions, p=[t.get_probability() for t in transitions])

                time_final = datetime.datetime.now()
                self.logger.log_time_elapsed("lrtdp_tvma_trial::time for transition selection", (time_final - time_initial).total_seconds())
                time_initial = datetime.datetime.now()
                state = self.mdp.compute_next_state(state, transition_selected)
                time_final = datetime.datetime.now()
                self.logger.log_time_elapsed("lrtdp_tvma_trial::time for next state computation", (time_final - time_initial).total_seconds())
                # print("next state: ", state.to_string())
            time_initial = datetime.datetime.now()
            while visited:
                state = visited.pop()
                # print("in while 2")
                if not self.check_solved(state, thetaparameter):
                    # print("State not solved: ", state.to_string(), "======", self.residual(state), "GOAL?  ", self.goal(state))
                    # print(len(visited), "states in visited stack")
                    break
            time_final = datetime.datetime.now()
            self.logger.log_time_elapsed("lrtdp_tvma_trial::time for backward check", (time_final - time_initial).total_seconds())

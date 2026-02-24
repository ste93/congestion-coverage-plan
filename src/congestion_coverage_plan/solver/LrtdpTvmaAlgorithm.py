import math

from congestion_coverage_plan.mdp.MDP import MDP, State
import datetime
from scipy.sparse import csr_array
from scipy.sparse.csgraph import shortest_path, minimum_spanning_tree
import numpy as np
from congestion_coverage_plan.utils import Logger
from congestion_coverage_plan.solver.Heuristics import Heuristics
import sys
from array import *

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
                 initial_state=None, 
                 logger=None,
                 explain_time=20):
        self.occupancy_map = occupancy_map

        self.mdp = MDP(occupancy_map=occupancy_map, 
                       time_for_occupancies=time_for_occupancies, 
                       time_start=time_start, 
                       wait_time=wait_time, 
                       explain_time=explain_time)
        self._wait_time = wait_time
        self.initial_time = time_for_occupancies
        self.time_for_occupancies = time_for_occupancies
        if initial_state is not None:
            self.vinitState = initial_state
            self.initial_time = time_for_occupancies
        else:
            self.vinitState = State(vertex=initial_state_name, 
                                    time=0, 
                                    visited_vertices=set([initial_state_name]), 
                                    pois_explained=set())
        self.vinitStateName = initial_state_name
        self.planning_time_bound = planning_time_bound
        self.solution_time_bound = solution_time_bound
        self.convergenceThresholdGlobal = convergence_threshold
        self.policy = {}
        self.policy_to_execute = {}
        self.valueFunction = {}
        self.action_costs = {}
        self.solved_set = set()
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
        print("congestion-coverage-plan init with version 2026-02-19")


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
            local_current_action_cost = 0
            local_current_action_cost = transition.get_cost() * transition.get_probability()
            current_action_cost = current_action_cost + local_current_action_cost

            next_state = self.mdp.compute_next_state(state, transition)
            local_future_actions_cost = self.get_value(next_state) * transition.get_probability()
            
            future_actions_cost = future_actions_cost + local_future_actions_cost
        cost = current_action_cost + future_actions_cost

        return cost


    def calculate_argmin_Q(self, state):
        qvalues = []
        state_internal = State(vertex=state.get_vertex(), 
                               time=state.get_time(), 
                               visited_vertices=state.get_visited_vertices().copy(), 
                               pois_explained=state.get_pois_explained().copy())
        possible_actions = self.mdp.get_possible_actions(state_internal)
        if not possible_actions:
            print("NO POSSIBLE ACTIONS - dead end reached for state:", state_internal.to_string())
            return (99999999, state_internal, "")
        time_initial = datetime.datetime.now()
        for action in possible_actions:
            qvalues.append((self.calculate_Q(state_internal, action), state_internal, action))

        min = None
        for qvalue in qvalues:
            if min is None:
                min = qvalue
            else:
                if qvalue[0] < min[0]:
                    min = qvalue
        return (min[0], min[1], min[2]) # this contains the value, state and action with the minimum Q value


    ### STATE FUNCTIONS
    def update(self, state, greedy_action=None):
        if greedy_action is None:
            greedy_action = self.greedy_action(state)
        self.valueFunction[state.to_string()] = greedy_action[0]  # this is the value of the state
        return True


    def greedy_action(self, state):
        return self.calculate_argmin_Q(state)


    def residual(self, state, greedy_action=None):
        if greedy_action is None:
            greedy_action = self.greedy_action(state)
        residual = abs(self.get_value(state) - greedy_action[0])
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


    def check_solved(self, state, thetaparameter, initial_greedy_action=None):
        """Check if state is solved - must explore all reachable states for correctness"""
        solved_condition = True
        open = []
        closed = set()  # Use set for O(1) membership checking
        closed_list = []  # Keep list to maintain states for update phase
        open.append((state, initial_greedy_action))  # Store state with optional precomputed action
        
        while open != []:
            state_entry = open.pop()
            state = state_entry[0]
            precomputed_greedy = state_entry[1]
            
            state_str = state.to_string()
            if state_str in closed:
                continue
                
            closed.add(state_str)
            closed_list.append(state)
            
            greedy = precomputed_greedy if precomputed_greedy is not None else self.greedy_action(state)
            if self.residual(state, greedy) > thetaparameter:
                solved_condition = False
                # Continue expanding - do NOT break (needed for correctness)

            action = greedy[2]
            for transition in self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound):
                next_state = self.mdp.compute_next_state(state, transition)
                next_str = next_state.to_string()
                if next_str not in closed and not self.solved(next_state):
                    open.append((next_state, None))  # No precomputed greedy for next states
                    
        if solved_condition:
            for state in closed_list:
                self.solved_set.add(state.to_string())
        else:
            for state in closed_list:
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


    def solve(self):
        number_of_trials = 0
        print("Predicting occupancies from time ", self.time_for_occupancies, " to ", self.time_for_occupancies + 100)
        initial_current_time_occupancies = datetime.datetime.now()
        self.occupancy_map.compute_current_tracks()
        print("Current tracks computed in ", (datetime.datetime.now() - initial_current_time_occupancies).total_seconds(), " seconds")
        initial_current_time_occupancies = datetime.datetime.now()
        self.occupancy_map.predict_occupancies(50)
        print("Occupancies predicted in ", (datetime.datetime.now() - initial_current_time_occupancies).total_seconds(), " seconds")
        initial_current_time_occupancies = datetime.datetime.now()
        self.occupancy_map.calculate_current_occupancies()
        print("current Occupancies predicted in ", (datetime.datetime.now() - initial_current_time_occupancies).total_seconds(), " seconds")
        initial_current_time = datetime.datetime.now()
        print("LRTDP TVMA started at: ", initial_current_time, "convergence threshold:", self.convergenceThresholdGlobal, "wait_time:", self._wait_time, "planner time bound:", self.solution_time_bound, "real time bound:", self.planning_time_bound, "initial time for occupancies:", self.time_for_occupancies)
        while (not self.solved(self.vinitState)) and ((datetime.datetime.now() - initial_current_time)) < datetime.timedelta(seconds = self.planning_time_bound):
            self.lrtdp_tvma_trial(self.vinitState, self.convergenceThresholdGlobal, self.solution_time_bound)
            number_of_trials += 1
            # print("trial number:", number_of_trials, "time elapsed:", (datetime.datetime.now() - initial_current_time).total_seconds(), "seconds")
            if number_of_trials % 50 == 0:
                print(len(self.policy), "states in policy")
                print(len(self.valueFunction), "states in value function")
        print("exit reason:", "solved" if self.solved(self.vinitState) else "time limit reached")
        print("number of trials:", number_of_trials)
        print("time elapsed:", (datetime.datetime.now() - initial_current_time).total_seconds(), "seconds")
        return self.solved(self.vinitState)


    def lrtdp_tvma_trial(self, vinitStateParameter, thetaparameter, solution_time_bound):
            visited = [] # this is a stack
            state = vinitStateParameter
            while not self.solved(state):
                visited.append(state)
                greedy = self.calculate_argmin_Q(state)  # Compute once
                self.update(state, greedy)  # Pass precomputed greedy action
                self.policy[state.to_string()] = greedy  # Reuse result
                if self.goal(state) or (state.get_time() > solution_time_bound):
                    ######## should there be here a bellamn backup?
                    break
                # perform bellman backup and update policy
                action = greedy[2]  # Use cached greedy action
                # print("state:", state.to_string(), "action:", action)
                transitions = self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound)
                if not transitions:
                    print("lrtdp_tvma_trial::No transitions found for state: ", state.to_string())
                    break
                transition_selected = np.random.choice(transitions, p=[t.get_probability() for t in transitions])
                old_state = state
                state = self.mdp.compute_next_state(state, transition_selected)
                self.policy_to_execute[old_state.to_string()] = (greedy[0], state, action)
                
                # Policy entries are (value, state, action); keep them immutable for callers.
                
            while visited:
                state = visited.pop()
                greedy = self.calculate_argmin_Q(state)  # Compute for check_solved
                if not self.check_solved(state, thetaparameter, greedy):
                    break  # Stop if state is not solved
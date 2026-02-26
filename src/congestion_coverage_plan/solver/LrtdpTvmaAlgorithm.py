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
        self.print_diagnostics_bool = False  # Set to True to enable detailed diagnostics
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
        # print(f"Calculating Q for state: {state.to_string()}, action: {action}")
        if self.goal(state):
            return 0
        
        current_action_cost = 0
        future_actions_cost = 0

        possible_transitions = self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound)
        if not possible_transitions:
            return self.solution_time_bound * 2
        # print(f"Possible transitions for state: {state.to_string()}, action: {action}:")
        for transition in possible_transitions:
            # print(f"Transition: {transition}, Probability: {transition.get_probability()}, Cost: {transition.get_cost()}")
            if transition.get_probability() == 0:
                continue
            local_current_action_cost = 0
            local_current_action_cost = transition.get_cost() * transition.get_probability()
            current_action_cost = current_action_cost + local_current_action_cost

            next_state = self.mdp.compute_next_state(state, transition)
            local_future_actions_cost = self.get_value(next_state) * transition.get_probability()
            
            future_actions_cost = future_actions_cost + local_future_actions_cost
        cost = current_action_cost + future_actions_cost
        # print("state:", state.to_string(), "action:", action, "future actions cost:", future_actions_cost, "current action cost:", current_action_cost, "total Q cost:", cost)
        # print(f"Q-value for state: {state.to_string()}, action: {action} is {cost} (current action cost: {current_action_cost}, future actions cost: {future_actions_cost})")
        return cost


    def calculate_argmin_Q(self, state):
        # Goal states have zero cost - don't try to compute actions
        if self.goal(state):
            return (0, state, None)
        
        # Time-limited horizon reached - if not at goal, return infinite cost (unsolvable)
        if state.get_time() > self.solution_time_bound:
            return (self.solution_time_bound * 2, state, None)
        
        qvalues = []
        state_internal = State(vertex=state.get_vertex(), 
                               time=state.get_time(), 
                               visited_vertices=state.get_visited_vertices().copy(), 
                               pois_explained=state.get_pois_explained().copy())
        possible_actions = self.mdp.get_possible_actions(state_internal)
        # print(f"Calculating argmin Q for state: {state_internal.to_string()}, possible actions: {possible_actions}")
        if not possible_actions:
            print("NO POSSIBLE ACTIONS - dead end reached for state:", state_internal.to_string())
            return (self.solution_time_bound * 2, state_internal, "")
        for action in possible_actions:
            q = self.calculate_Q(state_internal, action)
            qvalues.append((q, state_internal, action))
            # print(f"Action: {action}, Q-value: {q}")
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
        old_value = self.valueFunction.get(state.to_string(), None)
        if old_value is not None and (greedy_action[0] - old_value) > 100:
            print(f"!!! HUGE JUMP at {state.get_vertex()}: {old_value} -> {greedy_action[0]}")
            print(f"Action taken: {greedy_action[2]}")
        self.valueFunction[state.to_string()] = greedy_action[0]  # this is the value of the state
        self.print_diagnostics(f"update: State={state.to_string()}, OldValue={old_value}, NewValue={greedy_action[0]}, Action={greedy_action[2]}")
        return True


    def greedy_action(self, state):
        value, s, action = self.calculate_argmin_Q(state)
        self.print_diagnostics(f"greedy_action: State={state.to_string()}, Value={value}, Action={action}")
        return (value, s, action)

    def print_diagnostics(self, message):
        if self.print_diagnostics_bool:
            print(f"[DIAG] {message}")

    def residual(self, state, greedy_action=None):
        if greedy_action is None:
            greedy_action = self.greedy_action(state)
        residual = abs(self.get_value(state) - greedy_action[0])
        return residual


    def solved(self, state):
        return state.to_string() in self.solved_set


    def get_value(self, state):
        if state.to_string() in self.valueFunction:
            # print("Value function cache hit for state:", state.to_string(), "value:", self.valueFunction[state.to_string()])
            return self.valueFunction[state.to_string()]
        if state.to_string() in self.heuristic_backup:
            # print("Heuristic backup cache hit for state:", state.to_string(), "value:", self.heuristic_backup[state.to_string()])
            return self.heuristic_backup[state.to_string()]
        
        heuristic_value = self.heuristic_function(state)
        # print("Heuristic computed for state:", state.to_string(), "value:", heuristic_value)
        self.heuristic_backup[state.to_string()] = heuristic_value
        return heuristic_value


    def goal(self, state):
        is_goal = self.mdp.solved(state)
        # if is_goal:
        #     print("Goal reached at state:", state.to_string(), "time:", state.get_time())
        return is_goal


    def check_solved(self, state, thetaparameter, initial_greedy_action=None):
        """Check if state is solved - must explore all reachable states for correctness"""
        solved_condition = True
        open_list = []
        closed_set = set()  # Use set for O(1) membership checking
        closed_list = []  # Keep list to maintain states for update phase
        open_set = set()  # Track states in open to avoid duplicates
        
        open_list.append((state, initial_greedy_action))  # Store state with optional precomputed action
        open_set.add(state.to_string())
        
        while open_list != []:
            state_entry = open_list.pop()
            open_set.discard(state_entry[0].to_string())  # Remove from open tracking set
            state = state_entry[0]
            precomputed_greedy = state_entry[1]
            
            state_str = state.to_string()            
            closed_set.add(state_str)
            closed_list.append(state)
            
            residual_val = self.residual(state, None)
            
            # Never mark time-exceeded non-goal states as solved (they're infeasible)
            # if state.get_time() > self.solution_time_bound and not self.goal(state):
            #     solved_condition = False
            if residual_val > thetaparameter:
                solved_condition = False
                continue  # No need to expand further if residual is too high
                # Continue expanding - do NOT break (needed for correctness)
            greedy =  self.greedy_action(state)

            action = greedy[2]
            # Skip expansion for terminal states (goal or time-bounded states with action=None)
            # if action is None or self.goal(state) or state.get_time() > self.solution_time_bound:
            #     continue
                
            for transition in self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound):
                if transition.get_probability() > 0:
                    
                    next_state = self.mdp.compute_next_state(state, transition)
                    next_str = next_state.to_string()
                    # Only add if not solved, not in closed, and not already in open (matching pseudocode line 16)
                    if next_str not in closed_set and next_str not in open_set and not self.solved(next_state):
                        open_list.append((next_state, None))  # No precomputed greedy for next states
                        open_set.add(next_str)
        # print("check_solved: State=", state.to_string(), "SolvedCondition=", solved_condition, "VisitedStates=", len(closed_list))
        if solved_condition:
            for state in closed_list:
                self.solved_set.add(state.to_string())
        else:
            while closed_list:
                state = closed_list.pop()
                self.solved_set.discard(state.to_string())  # Remove from solved set if it was added

        
        return solved_condition


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
            # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            # print("current status after trial number:", number_of_trials, "time elapsed:", (datetime.datetime.now() - initial_current_time).total_seconds(), "seconds")
            # print("number of states in solved set:", len(self.solved_set))
            # print("number of states in value function:", len(self.valueFunction))
            # print("number of states in policy:", len(self.policy))
            # print("solved initial state:", self.solved(self.vinitState))
            # if not self.solved(self.vinitState):
            #     v = self.get_value(self.vinitState)
            #     q, _, _ = self.calculate_argmin_Q(self.vinitState)
            #     print(f"Initial State Value: {v}, Best Q: {q}, Delta: {abs(v-q)}")
            #     print("current value of initial state:", self.get_value(self.vinitState))
            #     print("residual of initial state:", self.residual(self.vinitState))
            #     print("reason for not being solved:")
            # # print("trial number:", number_of_trials, "time elapsed:", (datetime.datetime.now() - initial_current_time).total_seconds(), "seconds")
            if number_of_trials % 50 == 0:
                print(len(self.policy), "states in policy")
                print(len(self.valueFunction), "states in value function")
        print("exit reason:", "solved" if self.solved(self.vinitState) else "time limit reached")
        print("number of trials:", number_of_trials)
        print("time elapsed:", (datetime.datetime.now() - initial_current_time).total_seconds(), "seconds")
        return self.solved(self.vinitState)


    def lrtdp_tvma_trial(self, vinitStateParameter, thetaparameter, solution_time_bound):
        visited_list = []  # this is a stack
        visited_set = set()
        state = vinitStateParameter
        while not self.solved(state):
            visited_list.append(state)
            visited_set.add(state.to_string())

            if self.goal(state):
                # print(f"Goal reached at state {state.to_string()}")
                break
            if state.get_time() > solution_time_bound:
                # print(f"Time bound exceeded at state {state.to_string()}")
                break
            
            greedy = self.calculate_argmin_Q(state)  # Compute once
            self.update(state, greedy)  # Pass precomputed greedy action
            self.policy[state.to_string()] = greedy  # Reuse result
            # perform bellman backup and update policy
            # print("possible actions from state:", state.to_string(), "are:", self.mdp.get_possible_actions(state)   )
            action = greedy[2]  # Use cached greedy action

            if action is None:  # No valid action available
                print(f"No valid action at state {state.to_string()}")
                break

            transitions = self.mdp.get_possible_transitions_from_action(state, action, self.solution_time_bound)
            if not transitions:
                print(f"No transitions available from state {state.to_string()} with action {action}")
                break
            transition_selected = np.random.choice(transitions, p=[t.get_probability() for t in transitions])
            # print("transition selected:", transition_selected.get_action())
            state = self.mdp.compute_next_state(state, transition_selected)
        # print("Forward pass completed. Starting backward pass to check visited states.")
        # print("solved initial state? ", self.solved(self.vinitState))
        # print("number of visited states in this trial:", len(visited_list))
        # print("visited states:", [s.to_string() for s in visited_list])
        # Backward pass: check and update all visited states
        while visited_list:
            state = visited_list.pop()
            visited_set.discard(state.to_string())
            # Pass the precomputed greedy action from forward pass for consistency
            # precomputed_greedy = self.policy.get(state.to_string())

            # # First unsolved state found - print diagnostic
            # greedy = precomputed_greedy if precomputed_greedy else self.calculate_argmin_Q(state)
            if not self.check_solved(state, thetaparameter, None):
                break


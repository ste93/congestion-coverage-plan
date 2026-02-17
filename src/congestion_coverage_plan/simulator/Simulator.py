from matplotlib.pylab import matrix
from congestion_coverage_plan.mdp.MDP import MDP, State, Transition
from congestion_coverage_plan.solver.LrtdpTvmaAlgorithm import LrtdpTvmaAlgorithm
from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
import congestion_coverage_plan.utils.dataset_utils as dataset_utils
import warnings
from congestion_coverage_plan.tsp.tsp import *
from tqdm import *
from datetime import datetime
import congestion_coverage_plan.hamiltonian_path.hamiltonian_path as hamiltonian_path
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import create_data_model_from_matrix, solve_with_google, create_matrix_from_occupancy_map_length, create_matrix_from_occupancy_map_medium_occupancy, create_matrix_from_occupancy_map_current_occupancy, create_matrix_from_occupancy_map_high_occupancy, solve_with_google_with_data, compute_solution_cost
class Simulator:

    def __init__(self, occupancy_map, time_for_occupancies, wait_time, planning_time_bound, solution_time_bound, explain_time = 0):
        self._time_for_occupancies = time_for_occupancies # time offset to calculate occupancies
        self._occupancy_map = occupancy_map
        self._robot_min_speed = 0.6
        self._robot_max_speed = 1.2
        self._wait_time = wait_time # time added to the route when the robot decides to wait in a vertex in seconds
        self._explain_time = explain_time # time to explain the route in seconds
        self._planning_time_bound = planning_time_bound # maximum time for a single planning step in seconds
        self._solution_time_bound = solution_time_bound # maximum time for the whole route in seconds

    def set_time_for_occupancies(self, time):
        self._time_for_occupancies = time


    def execute_step(self,state, action):
        if action == "wait":
            # state, collisions, traverse_time
            return State(state.get_vertex(), state.get_time() + self._wait_time, state.get_visited_vertices().copy()), 0, self._wait_time
        if action == "explain":
            return State(state.get_vertex(), state.get_time() + self._explain_time, state.get_visited_vertices().copy()), 0, self._explain_time
        calculated_traverse_time, collisions = self.calculate_traverse_time(state, action)

        next_time = state.get_time() + calculated_traverse_time
        next_vertex = action
        # next_position = (self._occupancy_map.find_vertex_from_id(next_vertex).get_posx(), self._occupancy_map.find_vertex_from_id(next_vertex).get_posy())
        visited_vertices = state.get_visited_vertices().copy()
        if next_vertex not in state.get_visited_vertices():
            visited_vertices.add(next_vertex)
        next_state = State(next_vertex, next_time, visited_vertices)
        print(f"execute step from {state.get_vertex()} to {action}, traverse time: {calculated_traverse_time}, collisions: {collisions}, next state: {next_state}")
        return next_state, collisions, calculated_traverse_time
        

    def calculate_traverse_time(self, state, action):
        occupancies = self.get_current_occupancies(state)
        edge = self._occupancy_map.find_edge_from_position(state.get_vertex(), action)
        if edge is None:
            edge = self._occupancy_map.find_edge_from_position(action, state.get_vertex())
        if edge is None:
            return 0, 0  # Return default values if edge not found
        edge_name = edge.get_id()
        edge_occupancy = 0
        if edge_name in occupancies.keys():
            edge_occupancy = occupancies[edge_name]
        edge_traverse_time = self._occupancy_map.get_edge_traverse_time(edge_name)
        traverse_time = edge_traverse_time['zero'] + edge_occupancy* self._occupancy_map.get_people_collision_cost()
        return traverse_time, edge_occupancy


    def simulate_tsp_generic(self, start_time, initial_state, distance_matrix_function, robot_min_speed=None, robot_max_speed=None):
        policy = solve_with_google(self._occupancy_map, start_time, initial_state.get_vertex(), distance_matrix_function, time_bound=self._planning_time_bound)
        print("policy", policy)
        return self.simulate_tsp(start_time, initial_state, policy, robot_min_speed, robot_max_speed)


    def simulate_tsp(self, start_time, initial_state, policy, robot_min_speed = None, robot_max_speed = None):
        state = initial_state
        self.set_time_for_occupancies(start_time)
        if robot_max_speed is not None:
            self._robot_max_speed = robot_max_speed
        if robot_min_speed is not None:
            self._robot_min_speed = robot_min_speed
        steps = []
        steps_time = []

        prev_step = ""
        for vertex_name in policy[1:]:
            if (not self._occupancy_map.find_vertex_from_id(vertex_name) is None) and (prev_step == "" or not self._occupancy_map.find_vertex_from_id(prev_step) is None):
                
                state, collisions, traverse_time = self.execute_step(state, vertex_name)
                steps.append((vertex_name, collisions))
                steps_time.append(float(traverse_time))
                
            else:
                vertices_list = state.get_visited_vertices()
                vertices_list.add(vertex_name)

                state = State(vertex_name, state.get_time(), vertices_list)
            prev_step = vertex_name
        return (state.get_time(), steps, steps_time)


    def simulate_tsp_current_occupancy_with_replanning(self, start_time, initial_state):
        print("-------------------------------------simulate_tsp_current_occupancy_with_replanning----------------------------------")
        self.set_time_for_occupancies(start_time)
        completed = False
        state = initial_state
        # self._occupancy_map.predict_occupancies(time, 50)

        executed_steps = []
        planning_time = []

        steps_time = []
        while not completed:
            print("state before", state)
            # print("#####################################################################################")
            # print("init", self.get_current_occupancies(state))
            if len(state.get_visited_vertices()) == len(self._occupancy_map.get_vertices().keys()):
                completed = True
                break
            if state.get_time() > self._solution_time_bound:
                print("exit because state time exceeded time bound")
                print(state.get_visited_vertices())
                print(state.get_vertex())
                executed_steps.append(("FAILURE", 0))
                return (state.get_time(), executed_steps, steps_time)
            vertices_list = list(set(self._occupancy_map.get_vertices().keys()) - state.get_visited_vertices() | set([state.get_vertex()]))
            vertices_list.sort()
            print("vertices_list", vertices_list)
            map_current_occupancy = hamiltonian_path.create_matrix_from_vertices_list(
                vertices_ids=vertices_list,
                occupancy_map=self._occupancy_map,
                initial_vertex_id=state.get_vertex(),
                length_function=None
            )
            print("map_current_occupancy")
            for row in map_current_occupancy:
                print(row)
            data = create_data_model_from_matrix(map_current_occupancy)
            policy = hamiltonian_path.solve_with_google_with_data_returning_policy(data=data, vertex_list=vertices_list, time_bound=self._planning_time_bound)
            print("policy cost", compute_solution_cost(policy, self._occupancy_map))

            print("policy", policy)
            if policy is None:
                print("exit because no policy found")
                print(state.get_visited_vertices())
                print(state.get_vertex())
                executed_steps.append(("FAILURE", 0))
                return (state.get_time(), executed_steps, steps_time)
            else:
                print("found policy", policy)
                state, collisions, traverse_time = self.execute_step(state, policy[0])
                executed_steps.append((policy[0], collisions))
                steps_time.append(float(traverse_time))



                
        # print (state.get_time(), executed_steps)

        return (state.get_time(), executed_steps, steps_time)


    def simulate_lrtdp(self, start_time, initial_state, convergence_threshold, logger=None, simulate_planning_while_moving=False, heuristic_function=None):
        # print("start_time", start_time)
        self.set_time_for_occupancies(start_time)
        completed = False
        state = initial_state
        # self._occupancy_map.predict_occupancies(time, 50)

        executed_steps = []
        planning_time = []
        steps_time = []
        future_planning_time = self._planning_time_bound
        while not completed:
            # print("state before", state)
            # print("#####################################################################################")
            # print("init", self.get_current_occupancies(state))
            if len(state.get_visited_vertices()) == len(self._occupancy_map.get_vertices().keys()):
                completed = True
                break
            initial_planning_time = datetime.now()
            if not simulate_planning_while_moving:
                policy = self.plan(current_state=state, 
                                   planning_time_bound=self._planning_time_bound,
                                   solution_time_bound=self._solution_time_bound,
                                   logger=logger,
                                   convergence_threshold=convergence_threshold,
                                   heuristic_function=heuristic_function)
            else:
                policy = self.plan(current_state=state, 
                                   planning_time_bound=future_planning_time,
                                   solution_time_bound=self._solution_time_bound,
                                   logger=logger,
                                   convergence_threshold=convergence_threshold,
                                   heuristic_function=heuristic_function)
            total_planning_time = datetime.now() - initial_planning_time
            planning_time.append(float(total_planning_time.total_seconds()))
            # print(policy)
            # print("policy[0]", policy[0])
            # print("policy[1]", policy[1])
            if state.get_time() > self._solution_time_bound:
                print("exit because state time exceeded planner time bound")
                print(state.get_visited_vertices())
                print(state.get_vertex())
                executed_steps.append(("FAILURE", 0))
                return (state.get_time(), executed_steps, planning_time, steps_time)
            if policy[1] is not None:
                # print(policy)
                # print("policy for current state", policy[1][str(state)])
                if str(state) in policy[1]:
                    action = policy[1][str(state)]

                    print("action", action[2])
                    state, collisions, traverse_time = self.execute_step(state, action[2])
                    # print(state.get_time(), state.get_vertex())
                    future_planning_time = float(traverse_time)

                    executed_steps.append((action[2], collisions))
                    steps_time.append(float(traverse_time))
                    # print(state.get_time(), state.get_vertex())
                    print("state after", state)
                else:
                    print("exit because state not in policy[1], increasing planning time")
                    future_planning_time = future_planning_time + 10
                    
            else:
                if future_planning_time >= 200:
                    print("exit because policy[1] is none and future planning time too high")
                    print(state.get_visited_vertices())
                    print(state.get_vertex())
                    executed_steps.append(("FAILURE", 0))
                    return (state.get_time(), executed_steps , planning_time, steps_time)
                else:
                    future_planning_time = future_planning_time + 10
                    print("exit because policy[1] is none")
                    print(state.get_visited_vertices())
                    print(state.get_vertex())

                
        # print (state.get_time(), executed_steps)

        return (state.get_time(), executed_steps, planning_time, steps_time)


    def get_current_occupancies(self, state):
        current_time = self._time_for_occupancies + state.get_time()
        self._occupancy_map.calculate_current_occupancies(current_time)
        return self._occupancy_map.get_current_occupancies(current_time)


    def plan(self, current_state, logger, planning_time_bound, solution_time_bound, convergence_threshold, heuristic_function):
        # print("current_state", current_state)
        # print("start_time", self._start_time)
        # print("planning time", self._time_for_occupancies,  current_state.get_time())
        # print("planning")x 
        init_time = datetime.now()
        lrtdp = LrtdpTvmaAlgorithm(occupancy_map=self._occupancy_map, 
                                   initial_state_name=current_state.get_vertex(), 
                                   convergence_threshold=convergence_threshold, 
                                   planning_time_bound=planning_time_bound, 
                                   solution_time_bound=solution_time_bound, 
                                   time_for_occupancies=self._time_for_occupancies + current_state.get_time(),
                                   time_start=current_state.get_time(),
                                   wait_time=self._wait_time,
                                   vinitState=current_state, 
                                   logger=logger,
                                   heuristic_function=heuristic_function)
        # print("done creating")
        end_time = datetime.now()
        logger.log_time_elapsed("lrtdp_creation_time", (end_time - init_time).total_seconds())
        init_time = datetime.now()
        result = lrtdp.lrtdp_tvma()
        end_time = datetime.now()
        logger.log_time_elapsed("lrtdp_planning_time", (end_time - init_time).total_seconds())
        # print("Result---------------------------------------------------")
        # print(result)
        # print("lrtdp.policy", lrtdp.policy)
        # print("current_state", current_state)
        # --current vertex:-- vertex1 --current time:-- 0 --already visited vertices:--  vertex1
        # --current vertex:-- vertex1 --current time:-- 0 --already visited vertices:--  vertex1
        # print("lrtdp.policy.keys()", lrtdp.policy.keys())
        # print("lrtdp.policy[current_state]", lrtdp.policy[str(current_state)])
        # if not result:
        #     return (False, None)
        # if lrtdp.policy == {}:
        #     return (True, None)
        # print("lrtdp.policy", lrtdp.policy)
        # print("lrtdp.policy")
        # for x in lrtdp.policy:
        #     print(x)
        return (result, lrtdp.policy)



def simulate_tsp_with_policy(simulator, time, occupancy_map,  initial_state_name, policy, writer, file):
    initial_time = datetime.now()
    steps = simulator.simulate_tsp(time, State(initial_state_name,
                0, 
                set([initial_state_name])),
                policy)
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_with_policy", steps[0], len(steps[1]), time_used, 0, [float(time_used.total_seconds())], len(occupancy_map.get_occupancy_levels())])
    file.flush()



def simulate_tsp(simulator, time, occupancy_map,  initial_state_name, writer, file, simulate_steps_curr=True, simulate_steps_avg=True, simulate_steps_max=True, simulate_steps_min=True):
    initial_time = datetime.now()
    steps_curr = simulator.simulate_tsp_generic(time, State(initial_state_name,
                0, 
                set([initial_state_name])),
                create_matrix_from_occupancy_map_current_occupancy)
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_curr", steps_curr[0], steps_curr[1], time_used, steps_curr[2], [float(time_used.total_seconds())], len(occupancy_map.get_occupancy_levels())])
    file.flush()


    initial_time = datetime.now()
    steps_avg = simulator.simulate_tsp_generic(time, State(initial_state_name, 
                    0, 
                    set([initial_state_name])),
                    create_matrix_from_occupancy_map_medium_occupancy)
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_avg", steps_avg[0], steps_avg[1], time_used, steps_avg[2], [float(time_used.total_seconds())], len(occupancy_map.get_occupancy_levels())])
    file.flush()


    initial_time = datetime.now()
    steps_max = simulator.simulate_tsp_generic(time, State(initial_state_name, 
                    0,
                    set([initial_state_name])),
                    create_matrix_from_occupancy_map_high_occupancy)
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_max", steps_max[0], steps_max[1], time_used, steps_max[2], [float(time_used.total_seconds())], len(occupancy_map.get_occupancy_levels())])
    file.flush()


    initial_time = datetime.now()
    steps_min = simulator.simulate_tsp_generic(time, State(initial_state_name, 
                    0,
                    set([initial_state_name])),
                    create_matrix_from_occupancy_map_length)
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_min", steps_min[0], steps_min[1], time_used, steps_min[2], [float(time_used.total_seconds())], len(occupancy_map.get_occupancy_levels())])
    file.flush()


def simulate_tsp_current_occupancy_with_replanning(simulator, time, occupancy_map,  initial_state_name, writer, file):
    print("-------------------------------------tsp_current_occupancy_with_replanning----------------------------------")
    initial_time = datetime.now()
    steps_tsp_current_occupancy_with_replanning = simulator.simulate_tsp_current_occupancy_with_replanning(time, 
                                                                                                            State(initial_state_name, 
                                                                                                                0, 
                                                                                                                set([initial_state_name]))
                                                                                                            )
    print("=====================================end tsp_current_occupancy_with_replanning==============================")
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_tsp_current_occupancy_with_replanning", steps_tsp_current_occupancy_with_replanning[0], steps_tsp_current_occupancy_with_replanning[1], time_used, steps_tsp_current_occupancy_with_replanning[2], [float(time_used.total_seconds())], len(occupancy_map.get_occupancy_levels())])
    file.flush()

    
def simulate_lrtdp(simulator, time, occupancy_map,  initial_state_name, writer, file, logger, convergence_threshold, heuristic_function):
    print("-------------------------------------lrtdp----------------------------------")
    initial_time = datetime.now()
    steps_lrtdp = simulator.simulate_lrtdp(time, 
                                           State(initial_state_name, 
                                                0, 
                                                set([initial_state_name])), 
                                            convergence_threshold,
                                            logger, 
                                            simulate_planning_while_moving=False,
                                            heuristic_function=heuristic_function)
    print("=====================================end lrtdp==============================")
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_lrtdp", steps_lrtdp[0], steps_lrtdp[1], time_used, steps_lrtdp[3], steps_lrtdp[2], len(occupancy_map.get_occupancy_levels())])
    file.flush()


def simulate_lrtdp_planning_while_moving(simulator, time, occupancy_map,  initial_state_name, writer, file, logger, convergence_threshold, heuristic_function):
    print("-------------------------------------lrtdp----------------------------------")
    initial_time = datetime.now()
    steps_lrtdp = simulator.simulate_lrtdp(time, 
                                           State(initial_state_name, 
                                                0, 
                                                set([initial_state_name])), 
                                            convergence_threshold,
                                            logger,
                                            simulate_planning_while_moving=True,
                                            heuristic_function=heuristic_function)
    print("=====================================end lrtdp==============================")
    time_used = datetime.now() - initial_time
    writer.writerow([time, "steps_lrtdp_planning_while_moving", steps_lrtdp[0], steps_lrtdp[1], time_used, steps_lrtdp[3], steps_lrtdp[2], len(occupancy_map.get_occupancy_levels())])
    file.flush()
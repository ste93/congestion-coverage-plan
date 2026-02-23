import warnings
import csv
from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
from congestion_coverage_plan.simulator.Simulator import Simulator
from congestion_coverage_plan.simulator.Simulator import simulate_tsp, simulate_lrtdp, simulate_lrtdp_planning_while_moving, simulate_tsp_current_occupancy_with_replanning
from congestion_coverage_plan.utils import dataset_utils
from congestion_coverage_plan.utils import Logger
from tqdm import tqdm

def simulate_generic(filename, 
                     time_list, 
                     initial_state_name, 
                     predictor_creator_function, 
                     solution_time_bound, 
                     planning_time_bound, 
                     run_tsp_bool, 
                     run_lrtdp_bool, 
                     run_lrtdp_pwm_bool, 
                     run_tsp_bool_current_occupancy,
                     convergence_threshold, 
                     wait_time, 
                     heuristic_function,
                     explain_time, 
                     output_folder = None):
    print("arguments:")
    print("filename:", filename)
    print("time_list:", time_list)
    print("initial_state_name:", initial_state_name)
    print("solution_time_bound:", solution_time_bound)
    print("planning_time_bound:", planning_time_bound)
    print("run_tsp_bool:", run_tsp_bool)
    print("run_lrtdp_bool:", run_lrtdp_bool)
    print("run_lrtdp_pwm_bool:", run_lrtdp_pwm_bool)
    print("run_tsp_bool_current_occupancy:", run_tsp_bool_current_occupancy)
    print("convergence_threshold:", convergence_threshold)
    print("wait_time:", wait_time)
    print("heuristic_function:", heuristic_function)
    print("explain_time:", explain_time)
    base_folder = "results"
    warnings.filterwarnings("ignore")
    if output_folder is not None:
        base_folder = output_folder
    folder = base_folder + '/' + filename.split("/")[-1].split(".")[0]
    dataset_utils.create_folder(folder)
    # predictor = predictor_creator_function()
    # occupancy_map = OccupancyMap(predictor)
    # occupancy_map.load_occupancy_map(filename+ "_" + str(2) + "_levels.yaml")
    # occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size, occupancy_map.get_name())
    # simulator = Simulator(occupancy_map, 0)
    # simulate_lrtdp(simulator, time_list[0], occupancy_map, initial_state_name, None, None, solution_time_bound)
    writer_tsp = None
    writer_lrtdp = None
    writer_lrtdp_pwm = None
    writer_tsp_pwm = None
    # create files for results
    logger = Logger.Logger(print_time_elapsed=False)
    values_used_as_string = "_ptb_" + str(planning_time_bound) +  "_stb_" + str(solution_time_bound) + "_ctr_" + str(convergence_threshold).replace(".", "-")  + "_wtt_" + str(wait_time)  + "_heu_" + heuristic_function + "_expl_" + str(explain_time) + "_"
    filename_tsp = folder + "/" + filename.split("/")[-1].split(".")[0] + "-tsp-" + values_used_as_string + '.csv'
    filename_tsp_pwm = folder + "/" + filename.split("/")[-1].split(".")[0] + "-tsp_pwm-" + values_used_as_string + '.csv'
    filename_lrtdp = folder + "/" + filename.split("/")[-1].split(".")[0] + "-lrtdp-" + values_used_as_string + '.csv'
    filename_lrtdp_pwm = folder + "/" + filename.split("/")[-1].split(".")[0] + "-lrtdp_pwm-" + values_used_as_string + '.csv'
    if run_tsp_bool:
        with open(filename_tsp, 'w') as file_tsp:
            writer_tsp = csv.writer(file_tsp)
    if run_lrtdp_bool:
        with open(filename_lrtdp, 'w') as file_lrtdp:
            writer_lrtdp = csv.writer(file_lrtdp)
    if run_lrtdp_pwm_bool:
        with open(filename_lrtdp_pwm, 'w') as file_lrtdp_pwm:
            writer_lrtdp_pwm = csv.writer(file_lrtdp_pwm)
    if run_tsp_bool_current_occupancy:
        with open(filename_tsp_pwm, 'w') as file_tsp_pwm:
            writer_tsp_pwm = csv.writer(file_tsp_pwm)
        # for time in tqdm([0.0]):
        #     for level_number in range(2, 4):
    for time in tqdm(time_list):
        time = float(time)
        for level_number in [2,5,8]:
            
            if run_tsp_bool:
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                simulator = Simulator(occupancy_map=occupancy_map, 
                                      time_for_occupancies=0, 
                                      wait_time=wait_time, 
                                      planning_time_bound=planning_time_bound, 
                                      solution_time_bound=solution_time_bound, 
                                      explain_time=explain_time)

                with open(filename_tsp, 'a') as file_tsp:
                    writer_tsp = csv.writer(file_tsp)
                    print("Simulating TSP for time:", time, "and level:", level_number)
                    simulate_tsp(simulator=simulator, 
                                 time=time, 
                                 occupancy_map=occupancy_map, 
                                 initial_state_name=initial_state_name, 
                                 writer=writer_tsp, 
                                 file=file_tsp)


            if run_lrtdp_bool:
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                simulator = Simulator(occupancy_map=occupancy_map, 
                                      time_for_occupancies=0, 
                                      wait_time=wait_time, 
                                      planning_time_bound=planning_time_bound, 
                                      solution_time_bound=solution_time_bound, 
                                      explain_time=explain_time)
                with open(filename_lrtdp, 'a') as file_lrtdp:
                    writer_lrtdp = csv.writer(file_lrtdp)
                    print("Simulating LRTDP TVMA for time:", time, "and level:", level_number)
                    simulate_lrtdp(simulator=simulator, 
                                   time=time, 
                                   occupancy_map=occupancy_map, 
                                   initial_state_name=initial_state_name, 
                                   writer=writer_lrtdp, 
                                   file=file_lrtdp, 
                                   logger=logger, 
                                   convergence_threshold=convergence_threshold,
                                   heuristic_function=heuristic_function)
                    

            if run_lrtdp_pwm_bool:
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                # occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size, occupancy_map.get_name())
                simulator = Simulator(occupancy_map=occupancy_map, 
                                      time_for_occupancies=0, 
                                      wait_time=wait_time, 
                                      planning_time_bound=planning_time_bound,
                                      solution_time_bound=solution_time_bound,
                                      explain_time=explain_time)
                with open(filename_lrtdp_pwm, 'a') as file_lrtdp_pwm:
                    writer_lrtdp_pwm = csv.writer(file_lrtdp_pwm)
                    print("Simulating LRTDP TVMA while moving for time:", time, "and level:", level_number)
                    simulate_lrtdp_planning_while_moving(simulator=simulator, 
                                                           time=time, 
                                                           occupancy_map=occupancy_map, 
                                                           initial_state_name=initial_state_name, 
                                                           writer=writer_lrtdp_pwm, 
                                                           file=file_lrtdp_pwm, 
                                                           logger=logger, 
                                                           convergence_threshold=convergence_threshold, 
                                                           heuristic_function=heuristic_function)


            if run_tsp_bool_current_occupancy:  
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                simulator = Simulator(occupancy_map=occupancy_map, 
                                      time_for_occupancies=0, 
                                      wait_time=wait_time, 
                                      planning_time_bound=planning_time_bound,
                                      solution_time_bound=solution_time_bound,
                                      explain_time=explain_time)

                with open(filename_tsp_pwm, 'a') as file_tsp_pwm:
                    writer_tsp_pwm = csv.writer(file_tsp_pwm)
                    print("HERE")
                    print("Simulating TSP with current occupancy for time:", time, "and level:", level_number)
                    simulate_tsp_current_occupancy_with_replanning(simulator=simulator, 
                                 time=time, 
                                 occupancy_map=occupancy_map, 
                                 initial_state_name=initial_state_name, 
                                 writer=writer_tsp_pwm, 
                                 file=file_tsp_pwm)
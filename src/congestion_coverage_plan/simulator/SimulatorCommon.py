import warnings
import csv
from congestion_coverage_plan.OccupancyMap import OccupancyMap
from congestion_coverage_plan.simulator.Simulator import Simulator
from congestion_coverage_plan.simulator.SimulatorTSP import simulate_tsp
from congestion_coverage_plan.simulator.SimulatorLRTDP import simulate_lrtdp, simulate_lrtdp_planning_while_moving
from congestion_coverage_plan.utils import dataset_utils
from congestion_coverage_plan.utils import Logger
from tqdm import tqdm



def simulate_generic(filename, 
                     time_list, 
                     initial_state_name, 
                     predictor_creator_function, 
                     time_bound_lrtdp, 
                     time_bound_real, 
                     run_tsp_bool, 
                     run_lrtdp_bool, 
                     run_lrtdp_pwm_bool, 
                    run_tsp_bool_current_occupancy,
                     convergence_threshold, 
                     wait_time, 
                     heuristic_function):
    print("arguments:")
    print("filename:", filename)
    print("time_list:", time_list)
    print("initial_state_name:", initial_state_name)
    print("time_bound_lrtdp:", time_bound_lrtdp)
    print("time_bound_real:", time_bound_real)
    print("run_tsp_bool:", run_tsp_bool)
    print("run_lrtdp_bool:", run_lrtdp_bool)
    print("run_lrtdp_pwm_bool:", run_lrtdp_pwm_bool)
    print("convergence_threshold:", convergence_threshold)
    print("wait_time:", wait_time)
    print("heuristic_function:", heuristic_function)

    warnings.filterwarnings("ignore")
    folder = 'results/' + filename.split("/")[-1].split(".")[0]
    dataset_utils.create_folder(folder)
    # predictor = predictor_creator_function()
    # occupancy_map = OccupancyMap(predictor)
    # occupancy_map.load_occupancy_map(filename+ "_" + str(2) + "_levels.yaml")
    # occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size, occupancy_map.get_name())
    # simulator = Simulator(occupancy_map, 0)
    # simulate_lrtdp(simulator, time_list[0], occupancy_map, initial_state_name, None, None, time_bound_lrtdp)
    writer_tsp = None
    writer_lrtdp = None
    writer_lrtdp_pwm = None
    writer_tsp_pwm = None
    # create files for results
    logger = Logger.Logger(print_time_elapsed=False)
    filename_tsp_pwm = folder + "/" + filename.split("/")[-1].split(".")[0] + '_tsp_pwm_current_occupancy.csv'
    filename_tsp = folder + "/" + filename.split("/")[-1].split(".")[0] + '_tsp.csv'
    filename_lrtdp = folder + "/" + filename.split("/")[-1].split(".")[0] + "_" + str(time_bound_real) +  "_" + str(time_bound_lrtdp) + "_" + str(convergence_threshold).replace(".", "-")  + "_" + str(wait_time) + "_" + heuristic_function + '_lrtdp.csv'
    filename_lrtdp_pwm = folder + "/" + filename.split("/")[-1].split(".")[0] + "_" + str(time_bound_real) +"_" + str(time_bound_lrtdp) + "_" + str(convergence_threshold).replace(".", "-")  + "_" + str(wait_time)  + "_" + heuristic_function + '_lrtdp_pwm.csv'
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
                simulator = Simulator(occupancy_map, 0, wait_time, time_bound_real)

                with open(filename_tsp, 'a') as file_tsp:
                    writer_tsp = csv.writer(file_tsp)
                    print("Simulating TSP for time:", time, "and level:", level_number)
                    simulate_tsp(simulator=simulator, 
                                 time=time, 
                                 occupancy_map=occupancy_map, 
                                 initial_state_name=initial_state_name, 
                                 writer=writer_tsp, 
                                 file=file_tsp, 
                                 time_bound=time_bound_real)


            if run_lrtdp_bool:
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                simulator = Simulator(occupancy_map, 0, wait_time, time_bound_real, explain_time=20)
                with open(filename_lrtdp, 'a') as file_lrtdp:
                    writer_lrtdp = csv.writer(file_lrtdp)
                    print("Simulating LRTDP TVMA for time:", time, "and level:", level_number)
                    simulate_lrtdp(simulator=simulator, 
                                   time=time, 
                                   occupancy_map=occupancy_map, 
                                   initial_state_name=initial_state_name, 
                                   writer=writer_lrtdp, 
                                   file=file_lrtdp, 
                                   planner_time_bound=time_bound_lrtdp, 
                                   logger=logger, 
                                   convergence_threshold=convergence_threshold,
                                   heuristic_function=heuristic_function)
                    

            if run_lrtdp_pwm_bool:
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                # occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size, occupancy_map.get_name())
                simulator = Simulator(occupancy_map, 0, wait_time, time_bound_real)
                with open(filename_lrtdp_pwm, 'a') as file_lrtdp_pwm:
                    writer_lrtdp_pwm = csv.writer(file_lrtdp_pwm)
                    print("Simulating LRTDP TVMA while moving for time:", time, "and level:", level_number)
                    simulate_lrtdp_planning_while_moving(simulator=simulator, 
                                                           time=time, 
                                                           occupancy_map=occupancy_map, 
                                                           initial_state_name=initial_state_name, 
                                                           writer=writer_lrtdp_pwm, 
                                                           file=file_lrtdp_pwm, 
                                                           planner_time_bound=time_bound_lrtdp, 
                                                           logger=logger, 
                                                           convergence_threshold=convergence_threshold, 
                                                           heuristic_function=heuristic_function)


            if run_tsp_bool_current_occupancy:  
                predictor = predictor_creator_function()
                occupancy_map = OccupancyMap(predictor)
                occupancy_map.set_logger(logger)
                occupancy_map.load_occupancy_map(filename+ "_" + str(level_number) + "_levels.yaml")
                simulator = Simulator(occupancy_map, 0, wait_time, time_bound_real)

                with open(filename_tsp_pwm, 'a') as file_tsp_pwm:
                    writer_tsp_pwm = csv.writer(file_tsp_pwm)
                    print("Simulating TSP with current occupancy for time:", time, "and level:", level_number)
                    simulate_tsp(simulator=simulator, 
                                 time=time, 
                                 occupancy_map=occupancy_map, 
                                 initial_state_name=initial_state_name, 
                                 writer=writer_tsp_pwm, 
                                 file=file_tsp_pwm, 
                                 time_bound=time_bound_real)

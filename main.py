import warnings
from tqdm import tqdm
from congestion_coverage_plan.simulator.Simulator import Simulator, simulate_tsp, simulate_lrtdp, simulate_lrtdp_planning_while_moving, simulate_tsp_current_occupancy_with_replanning
import csv
from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
from congestion_coverage_plan.mdp.MDP import State
from congestion_coverage_plan.cliff_predictor.PredictorCreator import create_iit_cliff_predictor, create_atc_cliff_predictor, create_madama_cliff_predictor
import sys
from congestion_coverage_plan.utils import dataset_utils
from congestion_coverage_plan.tsp.tsp import *
from congestion_coverage_plan.utils import Logger
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import * 
from congestion_coverage_plan.simulator.SimulatorCommon import simulate_generic
from times import get_times_atc







def create_atc_with_name(filename, 
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
                         output_folder = None,
                         times = None):
    time_list = []
    if times is not None:
        time_list = times
    else:
        time_list = get_times_atc()
    print("Selected times:", time_list)
    initial_state_name = "vertex1"
    predictor_creator_function = create_atc_cliff_predictor
    simulate_generic(filename=filename, 
                     time_list=time_list, 
                     initial_state_name=initial_state_name, 
                     predictor_creator_function=predictor_creator_function, 
                     solution_time_bound=solution_time_bound, 
                     planning_time_bound=planning_time_bound, 
                     run_tsp_bool=run_tsp_bool, 
                     run_lrtdp_bool=run_lrtdp_bool, 
                     run_lrtdp_pwm_bool=run_lrtdp_pwm_bool, 
                     run_tsp_bool_current_occupancy=run_tsp_bool_current_occupancy,
                     convergence_threshold=convergence_threshold, 
                     wait_time=wait_time,
                     heuristic_function=heuristic_function,
                     explain_time=explain_time,
                     output_folder=output_folder)

def create_madama_with_name(filename, 
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
                         output_folder = None,
                         times = None):
    time_list = []
    with open('data/datasets/madama/madama_reduced_decimals.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            time_list.append(row[0])
    selected_time_list = []
    if times is not None:
        selected_time_list = times
    else:
        for time_index in tqdm(range(0, len(time_list), 5450)):
            selected_time_list.append(time_list[time_index])
    initial_state_name = "vertex1"
    print("Selected times:", selected_time_list)
    predictor_creator_function = create_madama_cliff_predictor
    # simulate_generic(filename, [0], initial_state_name, predictor_creator_function, solution_time_bound, run_tsp_bool, run_lrtdp_bool, run_lrtdp_pwm_bool, convergence_threshold)
    simulate_generic(filename=filename, 
                     time_list= selected_time_list, 
                     initial_state_name=initial_state_name, 
                     predictor_creator_function=predictor_creator_function, 
                     solution_time_bound=solution_time_bound, 
                     planning_time_bound=planning_time_bound, 
                     run_tsp_bool=run_tsp_bool, 
                     run_lrtdp_bool=run_lrtdp_bool, 
                     run_lrtdp_pwm_bool=run_lrtdp_pwm_bool, 
                     run_tsp_bool_current_occupancy=run_tsp_bool_current_occupancy,
                     convergence_threshold=convergence_threshold, 
                     wait_time=wait_time,
                     heuristic_function=heuristic_function,
                     explain_time=explain_time,
                     output_folder=output_folder)

def print_usage():
    print("Usage:")
    print("python main.py run --map [map_name] --algorithms [tsp] [lrtdp] [lrtdp_pwm] --convergence_threshold [convergence_threshold] --wait_time [wait_time] --solution_time_bound [solution_time_bound] --planning_time_bound [planning_time_bound] --heuristic [heuristic_function] --results_folder [results_folder]")
    print("python main.py show --map [map_name] --show_vertex_names")
    print("python main.py save --map [map_name] --show_vertex_names")
    print("python main.py create --occupancy_map_definition_name [name] --base_folder [folder] --occupancy_levels [levels] --cliff_predictor_name [name]")



if __name__ == "__main__":
    # if the argument is the name of the function, then execute the function
    args = sys.argv[1:]
    map_name = ""
    if "--map" in args:
        map_index = args.index("--map")
        if map_index + 1 < len(args):
            map_name = args[map_index + 1]
        else:
            print("Error: --map option requires a value.")
            print_usage()
            sys.exit(1)
    else:
        print("Error: --map option is required.")
        print_usage()
        sys.exit(1)

    show_vertex_names = False

    predictor = None
    if "atc" in map_name:
        predictor = create_atc_cliff_predictor()
    elif "madama" in map_name:
        predictor = create_madama_cliff_predictor()
    if args[-1] == "--show_vertex_names":
        show_vertex_names = True


    if len(args) > 2 and args[0] == "run":
        run_tsp_bool = False
        run_lrtdp_bool = False
        run_lrtdp_pwm_bool = False
        run_tsp_bool_current_occupancy = False
        convergence_threshold = 2.5
        wait_time = 10
        explain_time = 0
        solution_time_bound = 350
        planning_time_bound = 300
        times = None
        if "--algorithms" in args:
            print("Algorithms specified.")
            algorithms_index = args.index("--algorithms")
            for alg in args[algorithms_index + 1:]:
                if alg == "tsp":
                    run_tsp_bool = True
                elif alg == "lrtdp":
                    run_lrtdp_bool = True
                elif alg == "lrtdp_pwm":
                    run_lrtdp_pwm_bool = True
                elif alg == "tsp_pwm":
                    run_tsp_bool_current_occupancy = True
                else:
                    break
        else:
            run_tsp_bool_current_occupancy = True
            run_tsp_bool = True
            run_lrtdp_bool = True
            run_lrtdp_pwm_bool = True


        if "--convergence_threshold" in args:
            convergence_index = args.index("--convergence_threshold")
            if convergence_index + 1 < len(args):
                convergence_threshold = float(args[convergence_index + 1])

        if "--wait_time" in args:
            wait_time_index = args.index("--wait_time")
            if wait_time_index + 1 < len(args):
                wait_time = int(args[wait_time_index + 1])
        
        if "--explain_time" in args:
            explain_time_index = args.index("--explain_time")
            if explain_time_index + 1 < len(args):
                explain_time = int(args[explain_time_index + 1])

        if "--solution_time_bound" in args:
            solution_time_bound_index = args.index("--solution_time_bound")
            if solution_time_bound_index + 1 < len(args):
                solution_time_bound = int(args[solution_time_bound_index + 1])

        if "--planning_time_bound" in args:
            planning_time_bound_index = args.index("--planning_time_bound")
            if planning_time_bound_index + 1 < len(args):
                planning_time_bound = int(args[planning_time_bound_index + 1])
        
        if "--times" in args:
            times_index = args.index("--times")
            times = []
            for time_str in args[times_index + 1:]:
                try:
                    time_val = float(time_str)
                    times.append(time_val)
                except ValueError:
                    print("Invalid time value:", time_str)
                    break

        if "--times_file" in args:
            times_file_index = args.index("--times_file")
            if times_file_index + 1 < len(args):
                times_file = args[times_file_index + 1]
                times = []
                with open(times_file, 'r') as file:
                    reader = csv.reader(file)
                    for row in reader:
                        try:
                            time_val = float(row[0])
                            times.append(time_val)
                        except ValueError:
                            print("Invalid time value in file:", row[0])
            else:
                print("Error: --times_file option requires a value.")
                print_usage()
                sys.exit(1)

        if "--heuristic" in args:
            heuristic_index = args.index("--heuristic")
            if heuristic_index + 1 < len(args):
                heuristic_function = args[heuristic_index + 1]
            else:
                print("Error: --heuristic option requires a value.")
                print_usage()
                sys.exit(1)
        else:
            print("No heuristic function specified, need to specify one with --heuristic [function_name]")
            print_usage()
            sys.exit(1)


        if "--results_folder" in args:
            results_folder_index = args.index("--results_folder")
            if results_folder_index + 1 < len(args):
                results_folder = args[results_folder_index + 1]
                # Logger.set_results_folder(results_folder)
            else:
                print("Error: --results_folder option requires a value.")
                print_usage()
                sys.exit(1)
        else:
            print("No results folder specified, need to specify one with --results_folder [folder_name]")
            print_usage()
            sys.exit(1)

        print(times)
        path = "data/occupancy_maps/occupancy_maps_" + map_name + "/occupancy_map_" + map_name

        arg = map_name
        if "atc" in map_name:
            create_atc_with_name(filename=path, 
                                 solution_time_bound=solution_time_bound, 
                                 planning_time_bound=planning_time_bound,
                                 run_tsp_bool=run_tsp_bool, 
                                 run_lrtdp_bool=run_lrtdp_bool, 
                                 run_lrtdp_pwm_bool=run_lrtdp_pwm_bool, 
                                 run_tsp_bool_current_occupancy=run_tsp_bool_current_occupancy,
                                 convergence_threshold=convergence_threshold, 
                                 wait_time=wait_time,
                                 heuristic_function=heuristic_function,
                                 times=times, 
                                 explain_time=explain_time,
                                 output_folder=results_folder)
        elif "madama" in map_name:
            create_madama_with_name(filename=path, 
                                    solution_time_bound=solution_time_bound, 
                                    planning_time_bound=planning_time_bound,
                                    run_tsp_bool=run_tsp_bool, 
                                    run_lrtdp_bool=run_lrtdp_bool, 
                                    run_lrtdp_pwm_bool=run_lrtdp_pwm_bool, 
                                    run_tsp_bool_current_occupancy=run_tsp_bool_current_occupancy,
                                    convergence_threshold=convergence_threshold, wait_time=wait_time, 
                                    heuristic_function=heuristic_function,
                                    times=times, 
                                    explain_time=explain_time,
                                    output_folder=results_folder)
        else:
            print("Function not found: ", arg)
            print_usage()
            sys.exit(1)



    elif len(args) >= 2 and args[0] == "show":
        path = "data/occupancy_maps/occupancy_maps_" + map_name + "/occupancy_map_" + map_name + "_2_levels.yaml"
        print("Loading occupancy map from:", path)
        occupancy_map = OccupancyMap(predictor)
        occupancy_map.load_occupancy_map(path)
        occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size, "", True) # occupancy_map.get_name())
        occupancy_map.display_topological_map()


    elif len(args) >= 2 and args[0] == "show_cliff":
        path = "data/occupancy_maps_" + map_name + "/occupancy_map_" + map_name + "_2_levels.yaml"
        print("Loading occupancy map from:", path)
        predictor.display_cliff_map()


    elif len(args) >= 2 and args[0] == "save":
        path = "data/occupancy_maps/occupancy_maps_" + map_name + "/occupancy_map_" + map_name + "_2_levels.yaml"
        print("Loading occupancy map from:", path)
        occupancy_map = OccupancyMap(predictor)
        occupancy_map.load_occupancy_map(path)
        occupancy_map.plot_topological_map(predictor.map_file, predictor.fig_size,"") # occupancy_map.get_name())
        occupancy_map.save_figure(occupancy_map.get_name() + ".png")


    elif len(args) >= 2 and args[0] == "create":
        # inputs required are: occupancy_map_definition_name, base_folder, occupancy_levels, cliff_predictor_name
        pass 


    else:
        print("Invalid arguments.")
        print_usage()
        sys.exit(1)


from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import create_matrix_from_vertices_list, create_data_model_from_matrix, solve_with_google_with_data_returning_policy
from congestion_coverage_plan.hamiltonian_path.hamiltonian_path import INFINITE_DISTANCE
from congestion_coverage_plan.cliff_predictor.CliffPredictor import CliffPredictor
import PredictorCreator
import datetime

def test_create_matrix_from_vertices_list(occupancy_map_path, predictor_function):
    predictor = predictor_function()
    initial_vertex_id = "vertex1"
    occupancy_map = OccupancyMap(predictor)
    occupancy_map.load_occupancy_map(occupancy_map_path)
    print("initial_vertex_id", initial_vertex_id)
    # print("occupancy_map.get_vertices()", occupancy_map.get_vertices())
    # print("occupancy_map.get_vertices().keys()", occupancy_map.get_vertices().keys())
    vertices_ids = list(occupancy_map.get_vertices().keys())
    # print("vertices_ids", vertices_ids)
    vertices_ids.sort()
    matrix = create_matrix_from_vertices_list(vertices_ids, occupancy_map, initial_vertex_id, time_for_occupancies=0)
    data = create_data_model_from_matrix(matrix)
    policy = solve_with_google_with_data_returning_policy(data=data, vertex_list=vertices_ids, time_bound=1000)


if __name__ == "__main__":
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_madama_11/occupancy_map_madama_11_2_levels.yaml", PredictorCreator.create_madama_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 11 madama:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_madama_doors_26/occupancy_map_madama_doors_26_2_levels.yaml", PredictorCreator.create_madama_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 26 madama:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_madama_doors_21/occupancy_map_madama_doors_21_2_levels.yaml", PredictorCreator.create_madama_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 21 madama:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_madama_doors_16/occupancy_map_madama_doors_16_2_levels.yaml", PredictorCreator.create_madama_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 16 madama:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_atc_corridor_26/occupancy_map_atc_corridor_26_2_levels.yaml", PredictorCreator.create_atc_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 26 atc:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_atc_corridor_21/occupancy_map_atc_corridor_21_2_levels.yaml", PredictorCreator.create_atc_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 21 atc:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_atc_corridor_16/occupancy_map_atc_corridor_16_2_levels.yaml", PredictorCreator.create_atc_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 16 atc:", time_end - time_init)
    time_init = datetime.datetime.now()
    test_create_matrix_from_vertices_list("data/occupancy_maps/occupancy_maps_atc_corridor_11/occupancy_map_atc_corridor_11_2_levels.yaml", PredictorCreator.create_atc_cliff_predictor)
    time_end = datetime.datetime.now()
    print("Time taken 11 atc:", time_end - time_init)
import congestion_coverage_plan.utils.dataset_utils as dataset_utils
from congestion_coverage_plan.cliff_predictor.PredictorCreator import create_atc_cliff_predictor,  create_madama_cliff_predictor
import warnings
from OccupancyMapCreator import OccupancyMapCreator

from pyparsing import abstractmethod


class OccupancyMapDefinition:
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        self.name = name
        self.occupancy_map = occupancy_map
        self.occupancy_levels = occupancy_levels
        self.base_folder = base_folder
        self.occupancy_map.set_name(name)

    @abstractmethod
    def vertex_creation_function(self, occupancy_map):
        pass

    @abstractmethod
    def edge_creation_function(self, occupancy_map):
        pass


# --- MADAMA DOORS USED FOR REAL ----


class MadamaDoors16OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama3_doors_16_experiments_' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        self.occupancy_map.add_vertex_with_id("vertex1", -0.15, 0.39)  # start poi
        self.occupancy_map.add_vertex_with_id("vertex2", 6.42, -2.93, poi_number=1)
        self.occupancy_map.add_vertex_with_id("vertex3", 7.27, 1.47, poi_number=1)
        self.occupancy_map.add_vertex_with_id("vertex4", 17.05, 2.1, poi_number=2)
        self.occupancy_map.add_vertex_with_id("vertex5", 19.05, -3.0, poi_number=2)
        self.occupancy_map.add_vertex_with_id("vertex6", 15.68, -15.96, poi_number=3)
        self.occupancy_map.add_vertex_with_id("vertex7", 25.65, -15.88, poi_number=3)
        self.occupancy_map.add_vertex_with_id("vertex8", 20.15, -31.55, poi_number=4)
        self.occupancy_map.add_vertex_with_id("vertex9", 17.72, -33.59, poi_number=4)
        self.occupancy_map.add_vertex_with_id("vertex10", 1.16, -29.4, poi_number=5)
        self.occupancy_map.add_vertex_with_id("vertex11", 4.97, -28.47, poi_number=5)
        self.occupancy_map.add_vertex_with_id("vertex12", 13.94, -0.68) # door 1
        self.occupancy_map.add_vertex_with_id("vertex13", 16.09, -6.91) # door 2
        self.occupancy_map.add_vertex_with_id("vertex14", 15.65, -28.25) # door 3
        self.occupancy_map.add_vertex_with_id("vertex15", 12.09, -33.66) # door 4
        self.occupancy_map.add_vertex_with_id("vertex16", 0.97, -26.7, is_final_goal=True)  # final goal
    

    def edge_creation_function(self):
        # start poi
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")

        # first door
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex5")

        # second door

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex7")

        # third door

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex9")

        # fourth door

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex11")

        # final poi

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex16")


# ---- ATC CORRIDOR ----


class ATCCorridor6OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'atc_corridor_6' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        self.occupancy_map.add_vertex_with_id("vertex1", 50.55, -26.44)
        self.occupancy_map.add_vertex_with_id("vertex2", 42.88, -24.13)
        self.occupancy_map.add_vertex_with_id("vertex3", 52.28, -23.13)
        self.occupancy_map.add_vertex_with_id("vertex4", 47.14, -20.26)
        self.occupancy_map.add_vertex_with_id("vertex5", 41.46, -21.41)
        self.occupancy_map.add_vertex_with_id("vertex6", 40.03, -18.72)

    def edge_creation_function(self):
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex1")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex1")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex3")



        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex5")


class ATCCorridor11OccupancyMapDefinition(ATCCorridor6OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'atc_corridor_11' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex7", 35.64, -21.17)
        self.occupancy_map.add_vertex_with_id("vertex8", 34.15, -17.47)
        self.occupancy_map.add_vertex_with_id("vertex9", 31.7, -18.0)
        self.occupancy_map.add_vertex_with_id("vertex10", 27.88, -18.22)
        self.occupancy_map.add_vertex_with_id("vertex11", 27.0, -15.14)

    def edge_creation_function(self):
        super().edge_creation_function()
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex2")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex5")
        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex6")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex7")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex8")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex9")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex10")


class ATCCorridor16OccupancyMapDefinition(ATCCorridor11OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'atc_corridor_16' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex12", 24.71, -16.0)
        self.occupancy_map.add_vertex_with_id("vertex13", 21.91, -16.2)
        self.occupancy_map.add_vertex_with_id("vertex14", 22.6, -11.38)
        self.occupancy_map.add_vertex_with_id("vertex15", 19.79, -13.51)
        self.occupancy_map.add_vertex_with_id("vertex16", 17.02, -14.14)


    def edge_creation_function(self):
        super().edge_creation_function()
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex10")
        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex11")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex12")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex16")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex13")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex14")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex16")

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex15")


class ATCCorridor21OccupancyMapDefinition(ATCCorridor16OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'atc_corridor_21' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex17", 20.6, -9.38)
        self.occupancy_map.add_vertex_with_id("vertex18", 16.59, -9.92)
        self.occupancy_map.add_vertex_with_id("vertex19", 11.28, -8.67)
        self.occupancy_map.add_vertex_with_id("vertex20", 17.08, -5.88)
        self.occupancy_map.add_vertex_with_id("vertex21", 11.5, -5.21)

    def edge_creation_function(self):
        super().edge_creation_function()

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex18")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex14")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex18")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex15")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex19")


        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex16")

        # ----------------------------


        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex17")


        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex18")


        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex19")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex20")


class ATCCorridor26OccupancyMapDefinition(ATCCorridor21OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'atc_corridor_26' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex22", 7.11, -6.31)
        self.occupancy_map.add_vertex_with_id("vertex23", 11.10, -0.89)
        self.occupancy_map.add_vertex_with_id("vertex24", 7.00, -3.07)
        self.occupancy_map.add_vertex_with_id("vertex25", 6.80, 0.42)
        self.occupancy_map.add_vertex_with_id("vertex26", -0.09, -5.91)

    def edge_creation_function(self):
        super().edge_creation_function()

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex19")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex21")


        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex22")

        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex23")

        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex23")

        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex24")

        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex22")

        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex24")


        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex26")
        
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex25")


# ---- MADAMA FULL ----


class MadamaTopologicalMap11OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_11' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        self.occupancy_map.add_vertex_with_id("vertex1", 53.91, 26.05)
        self.occupancy_map.add_vertex_with_id("vertex2", 50.17, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex3", 54.12, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex4", 48.51, 43.02)
        self.occupancy_map.add_vertex_with_id("vertex5", 50.28, 50.69)
        self.occupancy_map.add_vertex_with_id("vertex6", 34.24, 41.64)
        self.occupancy_map.add_vertex_with_id("vertex7", 34.65, 50.53)
        self.occupancy_map.add_vertex_with_id("vertex8", 18.38, 45.31)
        self.occupancy_map.add_vertex_with_id("vertex9", 21.84, 41.71)
        self.occupancy_map.add_vertex_with_id("vertex10", 20.50, 28.87)
        self.occupancy_map.add_vertex_with_id("vertex11", 24.80, 29.86)

    def edge_creation_function(self):
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex1")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex8")
        
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex10")


class MadamaTopologicalMap16OccupancyMapDefinition(MadamaTopologicalMap11OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_16' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex12", 51.68, 30.24)
        self.occupancy_map.add_vertex_with_id("vertex13", 53.52, 42.5)
        self.occupancy_map.add_vertex_with_id("vertex14", 29.83, 46.08)
        self.occupancy_map.add_vertex_with_id("vertex15", 21.24, 48.84)
        self.occupancy_map.add_vertex_with_id("vertex16", 23.91, 34.07)

    def edge_creation_function(self):
        super().edge_creation_function()

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex1")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex3")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex12")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex2")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex5")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex13")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex5")

        # *** 

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex7")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex14")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex7")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex9")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex15")

        # ----------------------------

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex9")

        # ***

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex11")


class MadamaTopologicalMap21OccupancyMapDefinition(MadamaTopologicalMap16OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_21' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex17", 52.44, 34.06)
        self.occupancy_map.add_vertex_with_id("vertex18", 53.42, 45.28)
        self.occupancy_map.add_vertex_with_id("vertex19", 38.98, 44.86)
        self.occupancy_map.add_vertex_with_id("vertex20", 20.48, 43.66)
        self.occupancy_map.add_vertex_with_id("vertex21", 21.81, 32.66)

    def edge_creation_function(self):
        super().edge_creation_function()

        # ---------------------------- 17 connections
        # *** Previous room
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex1")

        # *** Current room

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex12")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex13")



        # ---------------------------- 18 connections
        # *** Previous room
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex17")
        
        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex13")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex18")

        # ---------------------------- 19 connections
        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex19")  
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex18")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex14")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex15")

        # ---------------------------- 20 connections

        # *** Previous room
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex19")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex15")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex16")

        # ---------------------------- 21 connections

        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex20")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex16")


class MadamaTopologicalMap26OccupancyMapDefinition(MadamaTopologicalMap21OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_26' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex22", 48.99, 36.83)
        self.occupancy_map.add_vertex_with_id("vertex23", 47.82, 48.35)
        self.occupancy_map.add_vertex_with_id("vertex24", 36.67, 46.54)
        self.occupancy_map.add_vertex_with_id("vertex25", 22.16, 45.91)
        self.occupancy_map.add_vertex_with_id("vertex26", 20.20, 34.83)

    def edge_creation_function(self):
        super().edge_creation_function()
        # ---------------------------- 22 connections
        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex1")

        # *** Current room

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex17")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex18")

        # ---------------------------- 23 connections

        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex22")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex18")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex19")

        # ---------------------------- 24 connections

        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex18")

        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex23")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex19")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex20")

        # ---------------------------- 25 connections

        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex19")

        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex24")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex20")

        # *** Next room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex16")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex21")

        # ---------------------------- 26 connections

        # *** Previous room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex25")

        # *** current room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex16")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex21")


# ---- MADAMA DOORS ----


class MadamaTopologicalMapDoors11OccupancyMapDefinition(MadamaTopologicalMap11OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_doors_11' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()

    def edge_creation_function(self):
        super().edge_creation_function()


class MadamaTopologicalMapDoors16OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_doors_16' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        self.occupancy_map.add_vertex_with_id("vertex1", 53.91, 26.05)
        self.occupancy_map.add_vertex_with_id("vertex2", 50.17, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex3", 54.12, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex4", 48.51, 43.02)
        self.occupancy_map.add_vertex_with_id("vertex5", 50.28, 50.69)
        self.occupancy_map.add_vertex_with_id("vertex6", 34.24, 41.64)
        self.occupancy_map.add_vertex_with_id("vertex7", 34.65, 50.53)
        self.occupancy_map.add_vertex_with_id("vertex8", 18.38, 45.31)
        self.occupancy_map.add_vertex_with_id("vertex9", 21.84, 41.71)
        self.occupancy_map.add_vertex_with_id("vertex10", 20.50, 28.87)
        self.occupancy_map.add_vertex_with_id("vertex11", 24.80, 29.86)
        self.occupancy_map.add_vertex_with_id("vertex12", 51.81, 39.04) # door 1
        self.occupancy_map.add_vertex_with_id("vertex13", 45.43, 41.42) # door 2
        self.occupancy_map.add_vertex_with_id("vertex14", 45.15, 49.20) # door 3
        self.occupancy_map.add_vertex_with_id("vertex15", 23.98, 40.09) # door 4
        self.occupancy_map.add_vertex_with_id("vertex16", 18.80, 37.43) # door 5


    def edge_creation_function(self):
        # start poi
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex1")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex1")

        # first room

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex2")

        # first door

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex12")

        # second room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex4")

        # second door

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex13")

        # third door

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex14")

        # third room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex6")

        # fourth door

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex15")

        # fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex8")

        # fifth door

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex16")

        # fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex10")


class MadamaTopologicalMapDoors21OccupancyMapDefinition(MadamaTopologicalMapDoors16OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_doors_21' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex17", 52.44, 34.06)
        self.occupancy_map.add_vertex_with_id("vertex18", 53.42, 45.28)
        self.occupancy_map.add_vertex_with_id("vertex19", 38.98, 44.86)
        self.occupancy_map.add_vertex_with_id("vertex20", 20.48, 43.66)
        self.occupancy_map.add_vertex_with_id("vertex21", 21.81, 32.66)


    def edge_creation_function(self):
        super().edge_creation_function()
        # 17 is in the first room

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex3")

        # 17 door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex12")

        # 18 is in the second room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex5")

        # 18 to first door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex12")

        # 18 to second door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex13")

        #18 to third door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex14")

        # 19 is in the third room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex7")

        # 19 to second door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex13")

        # 19 to third door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex14")

        # 19 to fourth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex15")

        # 20 is in the fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex9")

        # 20 to fourth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex15")

        # 20 to fifth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex16")

        # 21 is in the fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex11")

        # 21 to fifth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex16")


class MadamaTopologicalMapDoors26OccupancyMapDefinition(MadamaTopologicalMapDoors21OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_doors_26' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)


    def vertex_creation_function(self):
        super().vertex_creation_function()
        self.occupancy_map.add_vertex_with_id("vertex22", 48.99, 36.83)
        self.occupancy_map.add_vertex_with_id("vertex23", 49.82, 47.35)
        self.occupancy_map.add_vertex_with_id("vertex24", 31.67, 46.54)
        self.occupancy_map.add_vertex_with_id("vertex25", 18.16, 41.91)
        self.occupancy_map.add_vertex_with_id("vertex26", 23.20, 25.83)


    def edge_creation_function(self):
        super().edge_creation_function()
        # 22 is in the first room

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex17")

        # 22 to first door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex12")

        # 23 is in the second room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex18")


        # 23 to first door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex12")


        # 23 to second door connections


        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex13")


        # 23 to third door connections


        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex14")


        # 24 is in the third room

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex6")
        
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex19")

        # 24 to second door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex13")

        # 24 to third door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex14")

        # 24 to fourth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex15")

        # 25 is in the fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex20")

        # 25 to fourth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex15")

        # 25 to fifth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex16")

        # 26 is in the fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex21")

        # 26 to fifth door connections

        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex16")


# ---- MADAMA SEQUENTIAL ----


class MadamaTopologicalMapSequential11OccupancyMapDefinition(MadamaTopologicalMapDoors11OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_sequential_11' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        super().vertex_creation_function()

    def edge_creation_function(self):
        super().edge_creation_function()


class MadamaTopologicalMapSequential16OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_sequential_16' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):
        self.occupancy_map.add_vertex_with_id("vertex1", 53.91, 26.05)
        self.occupancy_map.add_vertex_with_id("vertex2", 50.17, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex3", 54.12, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex4", 48.51, 43.02)
        self.occupancy_map.add_vertex_with_id("vertex5", 50.28, 50.69)
        self.occupancy_map.add_vertex_with_id("vertex6", 34.24, 41.64)
        self.occupancy_map.add_vertex_with_id("vertex7", 34.65, 50.53)
        self.occupancy_map.add_vertex_with_id("vertex8", 18.38, 45.31)
        self.occupancy_map.add_vertex_with_id("vertex9", 21.84, 41.71)
        self.occupancy_map.add_vertex_with_id("vertex10", 20.50, 28.87)
        self.occupancy_map.add_vertex_with_id("vertex11", 24.80, 29.86)
        self.occupancy_map.add_vertex_with_id("vertex12", 42.92, 51.01)
        self.occupancy_map.add_vertex_with_id("vertex13", 41.92, 43.24)
        self.occupancy_map.add_vertex_with_id("vertex14", 27.66, 40.50)
        self.occupancy_map.add_vertex_with_id("vertex15", 26.6, 48.84)
        self.occupancy_map.add_vertex_with_id("vertex16", 23.6, 25.9)

    def edge_creation_function(self):
        # first room

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex2")


        # connections first and second room 

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex3")

        # second room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex4")

        # connections second and third room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex5")

        # third room

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex7")

        # connections third and fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex15")


        # fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex8")

        # connections fourth and fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex9")

        # fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex11")


class MadamaTopologicalMapSequential21OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_sequential_21' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creator_function(self):
        self.occupancy_map.add_vertex_with_id("vertex1", 53.91, 26.05)
        self.occupancy_map.add_vertex_with_id("vertex2", 50.17, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex3", 54.12, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex4", 48.51, 43.02)
        self.occupancy_map.add_vertex_with_id("vertex5", 50.28, 50.69)
        self.occupancy_map.add_vertex_with_id("vertex6", 34.24, 41.64)
        self.occupancy_map.add_vertex_with_id("vertex7", 34.65, 50.53)
        self.occupancy_map.add_vertex_with_id("vertex8", 18.38, 45.31)
        self.occupancy_map.add_vertex_with_id("vertex9", 21.84, 41.71)
        self.occupancy_map.add_vertex_with_id("vertex10", 20.50, 28.87)
        self.occupancy_map.add_vertex_with_id("vertex11", 24.80, 29.86)
        self.occupancy_map.add_vertex_with_id("vertex12", 42.92, 51.01)
        self.occupancy_map.add_vertex_with_id("vertex13", 41.92, 43.24)
        self.occupancy_map.add_vertex_with_id("vertex14", 27.66, 40.50)
        self.occupancy_map.add_vertex_with_id("vertex15", 26.6, 48.84)
        self.occupancy_map.add_vertex_with_id("vertex16", 23.6, 25.9)
        self.occupancy_map.add_vertex_with_id("vertex17", 49.20, 37.17) 
        self.occupancy_map.add_vertex_with_id("vertex18", 54.82, 37.35)
        self.occupancy_map.add_vertex_with_id("vertex19", 53.98, 42.86)
        self.occupancy_map.add_vertex_with_id("vertex20", 23.34,35.5)
        self.occupancy_map.add_vertex_with_id("vertex21", 19.12, 35.2)


    def edge_creation_function(self):
        # first room

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex17")

        # connections first and second room

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex18")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex18")

        # second room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex5")

        # connections second and third room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex5")

        # third room

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex14")

        # connections third and fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex8")
        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex15")

        # fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex8")

        # connections fourth and fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex9")

        # fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex11")


class MadamaTopologicalMapSequential26OccupancyMapDefinition(OccupancyMapDefinition):
    def __init__(self, name, occupancy_map, occupancy_levels, base_folder):
        if name is None:
            name = 'madama_topological_map_sequential_26' + str(len(occupancy_levels)) + '_occupancy_levels'
            print("No name provided for the occupancy map definition, using default name: " + name)
        super().__init__(name, occupancy_map, occupancy_levels, base_folder)

    def vertex_creation_function(self):

        self.occupancy_map.add_vertex_with_id("vertex1", 53.91, 26.05)
        self.occupancy_map.add_vertex_with_id("vertex2", 50.17, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex3", 54.12, 32.65)
        self.occupancy_map.add_vertex_with_id("vertex4", 48.51, 43.02)
        self.occupancy_map.add_vertex_with_id("vertex5", 50.28, 50.69)
        self.occupancy_map.add_vertex_with_id("vertex6", 34.24, 41.64)
        self.occupancy_map.add_vertex_with_id("vertex7", 34.65, 50.53)
        self.occupancy_map.add_vertex_with_id("vertex8", 18.38, 45.31)
        self.occupancy_map.add_vertex_with_id("vertex9", 21.84, 41.71)
        self.occupancy_map.add_vertex_with_id("vertex10", 20.50, 28.87)
        self.occupancy_map.add_vertex_with_id("vertex11", 24.80, 29.86)
        self.occupancy_map.add_vertex_with_id("vertex12", 42.92, 51.01)
        self.occupancy_map.add_vertex_with_id("vertex13", 41.92, 43.24)
        self.occupancy_map.add_vertex_with_id("vertex14", 27.66, 40.50)
        self.occupancy_map.add_vertex_with_id("vertex15", 26.6, 48.84)
        self.occupancy_map.add_vertex_with_id("vertex16", 23.6, 25.9)
        self.occupancy_map.add_vertex_with_id("vertex17", 49.20, 37.17) 
        self.occupancy_map.add_vertex_with_id("vertex18", 54.82, 37.35)
        self.occupancy_map.add_vertex_with_id("vertex19", 53.98, 42.86)
        self.occupancy_map.add_vertex_with_id("vertex20", 23.34,35.5)
        self.occupancy_map.add_vertex_with_id("vertex21", 19.12, 35.2)
        self.occupancy_map.add_vertex_with_id("vertex22", 54.35, 49.00)
        self.occupancy_map.add_vertex_with_id("vertex23", 34.25, 45.92)
        self.occupancy_map.add_vertex_with_id("vertex24", 20.9, 48.0)
        self.occupancy_map.add_vertex_with_id("vertex25", 22.07, 32.4)
        self.occupancy_map.add_vertex_with_id("vertex26", 52.0, 34.1)


    def edge_creation_function(self):
        # first room

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex2")
        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex3")
        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex1", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex1")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex2", "vertex17")
        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex2")

        # occupancy_map.add_edge_with_incremental_id("vertex3", "vertex17")
        # occupancy_map.add_edge_with_incremental_id("vertex17", "vertex3")

        # occupancy_map.add_edge_with_incremental_id("vertex2", "vertex18")
        # occupancy_map.add_edge_with_incremental_id("vertex18", "vertex2")

        self.occupancy_map.add_edge_with_incremental_id("vertex3", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex3")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex18")
        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex26")
        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex18")

        # connections first and second room

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex17", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex17")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex18")

        self.occupancy_map.add_edge_with_incremental_id("vertex18", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex18")

        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex4")
        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex26")

        self.occupancy_map.add_edge_with_incremental_id("vertex26", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex26")

        # second room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex5")
        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex19")
        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex19", "vertex22")
        self.occupancy_map.add_edge_with_incremental_id("vertex22", "vertex19")

        # connections second and third room

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex12")
        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex5")

        self.occupancy_map.add_edge_with_incremental_id("vertex4", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex4")

        self.occupancy_map.add_edge_with_incremental_id("vertex5", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex5")

        # third room

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex13")
        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex6")
        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex7")
        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex12", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex12")

        self.occupancy_map.add_edge_with_incremental_id("vertex13", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex13")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex14")
        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex6", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex6")

        self.occupancy_map.add_edge_with_incremental_id("vertex7", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex7")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex23")
        self.occupancy_map.add_edge_with_incremental_id("vertex23", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex15")
        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex14")

        # connections third and fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex14", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex14")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex15")

        self.occupancy_map.add_edge_with_incremental_id("vertex15", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex15")

        # fourth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex24")
        self.occupancy_map.add_edge_with_incremental_id("vertex24", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex9")
        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex8")

        # connections fourth and fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex8", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex8")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex20")
        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex9")

        self.occupancy_map.add_edge_with_incremental_id("vertex9", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex9")

        # fifth room

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex21")
        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex20")

        self.occupancy_map.add_edge_with_incremental_id("vertex20", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex20")


        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex10")
        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex21", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex21")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex11")
        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex25")
        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex10", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex10")

        self.occupancy_map.add_edge_with_incremental_id("vertex11", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex11")

        self.occupancy_map.add_edge_with_incremental_id("vertex25", "vertex16")
        self.occupancy_map.add_edge_with_incremental_id("vertex16", "vertex25")

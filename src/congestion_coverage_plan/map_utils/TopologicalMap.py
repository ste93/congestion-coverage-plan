# topological map representation of a location.
# The topological map is a dictionary with the following format:
# { 
#   'name': 'map_name',
#   'vertices': [{'id': uuidvertex1, 'posx': x1, 'posy': y1}, {'id': uuidvertex2, 'posx': x2, 'posy': y2},, ...],
#   'edges': [{'id': uuidedge1, 'start': uuidvertex1, 'end': uuidvertex2}, {'id': uuidedge2, 'start': uuidvertex2, 'end': uuidvertex3}, ...],
#   'goal_vertices': [uuidvertex1, uuidvertex2, ...]
# }
# The topological map can be saved and loaded to/from a yaml file.
# The topological map can be visualized using the function plot_topological_map.

import sys
import yaml
import uuid
import matplotlib.pyplot as plt
import matplotlib
import math


class Vertex:
    def __init__(self, id, posx, posy, is_poi = False, poi_number = None, is_final_goal = False):
        self._id = id
        self._posx = posx
        self._posy = posy
        self._is_poi = is_poi
        self._poi_number = poi_number
        self._is_final_goal = is_final_goal

    def __eq__(self, other):
        return self._id == other.get_id() and self._posx == other.get_posx() and self._posy == other.get_posy()

    def __hash__(self) -> int:
        return hash((self._id, self._posx, self._posy))
    
    def to_string(self):
        return str(self._id) + " " + str(self._posx) + " " + str(self._posy)
    # create getters for the class
    def get_id(self):
        return self._id
    
    def get_posx(self):
        return self._posx
    
    def get_posy(self):
        return self._posy

    def is_inside_area(self, x, y):
        # return if is inside a circle with center in the vertex and radius 1
        is_inside = (x - self._posx)**2 + (y - self._posy)**2 <= 1
        return is_inside
    
    def is_poi(self):
        return self._is_poi
    
    def get_poi_number(self):
        return self._poi_number

    def is_final_goal(self):
        return self._is_final_goal


class Edge:
    def __init__(self, id, start, end, start_x = None, start_y = None, end_x = None, end_y = None):
        self._id = id
        self._start = start
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self._end = end
        self.area = self.calculate_area()
        self.area_as_polygon = self.get_area_as_polygon()
        

    def __eq__(self, other):
        return self._id == other.get_id() and self._start == other.get_start() and self._end == other.get_end()
    
    def __hash__(self):
        return hash((self._id, self._start, self._end))
    
    def to_string(self):
        return str(self._id) + " " + str(self._start) + " " + str(self._end)

    # create getters for the class
    def get_id(self):
        return self._id
    
    def get_start(self):
        return self._start
    
    def get_end(self):
        return self._end
    
    def get_length(self):
        if self.start_x is None or self.start_y is None or self.end_x is None or self.end_y is None:
            return None
        return ((self.start_x - self.end_x)**2 + (self.start_y - self.end_y)**2)**0.5

    def calculate_area(self):
        edge_size_multiplier = 2
        if self.start_x == self.end_x:
            x1 = self.start_x - edge_size_multiplier
            y1 = self.start_y
            x2 = self.start_x + edge_size_multiplier
            y2 = self.start_y
            x3 = self.end_x + edge_size_multiplier
            y3 = self.end_y
            x4 = self.end_x - edge_size_multiplier
            y4 = self.end_y
            return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        if self.start_y == self.end_y:
            x1 = self.start_x
            y1 = self.start_y - edge_size_multiplier    
            x2 = self.start_x
            y2 = self.start_y + edge_size_multiplier
            x3 = self.end_x
            y3 = self.end_y + edge_size_multiplier  
            x4 = self.end_x
            y4 = self.end_y - edge_size_multiplier
            return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        if self.start_x is None or self.start_y is None or self.end_x is None or self.end_y is None:
            return None
        m = (self.start_y - self.end_y) / (self.start_x - self.end_x)
        m2 = -1/m
        delta_x = math.sqrt( edge_size_multiplier**2 / (1 + (m2**2)) ) / 2
        delta_y = m2 * delta_x
        x1 = self.start_x - delta_x
        y1 = self.start_y - delta_y
        x2 = self.start_x + delta_x
        y2 = self.start_y + delta_y
        x3 = self.end_x + delta_x
        y3 = self.end_y + delta_y
        x4 = self.end_x - delta_x
        y4 = self.end_y - delta_y
        return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]



        

    def get_area(self):
        return self.area


    def get_area_as_polygon(self):
        if self.area is None:
            return None
        return matplotlib.path.Path(self.area)




    def is_inside_area(self, x, y):
        return self.area_as_polygon.contains_point((x, y))


class TopologicalMap:
    def __init__(self):
        self.name = ""
        self.vertices = {}
        self.edges = {}
        self.goal_vertices = []
        self._vertices_list = []
        self.fig, self.ax = plt.subplots()
        self.edges_from_vertex = {}
        self.edges_class_from_vertex = {}
        self.pois_set = set()
        self.final_goal_vertices = set()

    ## SETTERS


    def set_name(self, name):
        self.name = name


    def compute_edges_from_vertex(self):
        self.edges_from_vertex = {}
        for vertex_id in self.vertices.keys():
            self.edges_from_vertex[vertex_id] = []
            self.edges_class_from_vertex[vertex_id] = {}
        for key_edge in self.edges.keys():
            self.edges_from_vertex[self.edges[key_edge].get_start()].append(self.edges[key_edge].get_end())
            self.edges_class_from_vertex[self.edges[key_edge].get_start()][self.edges[key_edge].get_end()] = self.edges[key_edge]
            # self.edges_from_vertex[edge.get_end()].append(edge)


    def compute_pois_set(self):
        self.pois_set = set()
        for vertex_id in self.vertices.keys():
            vertex = self.vertices[vertex_id]
            if vertex.is_poi():
                self.pois_set.add(vertex.get_poi_number())


    def compute_final_goal_vertices(self):
        self.final_goal_vertices = set()
        for vertex_id in self.vertices.keys():
            vertex = self.vertices[vertex_id]
            if vertex.is_final_goal():
                self.final_goal_vertices.add(vertex.get_id())

    ## GETTERS



    def get_final_goal_vertices(self):
        return self.final_goal_vertices

    def get_pois_set(self):
        return self.pois_set

    def get_vertex(self, vertex_id):
        return self.vertices[vertex_id]

    def get_vertices(self):
        return self.vertices
    
    def get_edges(self):   
        return self.edges
    
    def get_goal_vertices_list(self):
        return self.goal_vertices

    def get_name(self):
        return self.name
    
    ## Functions for setting the topological map

    ### Vertices functions
    def get_edges_from_vertex(self, vertex_id):
        return self.edges_from_vertex[vertex_id]




    def get_edges_from_vertex_with_edge_class(self, vertex_id):
        edges = []
        if self.edges_from_vertex.get(vertex_id) is None:
            return edges
        for edge_end in self.edges_from_vertex[vertex_id]:
            edges.append(self.find_edge_from_position(vertex_id, edge_end))
        return edges

    def add_vertex(self, posx, posy, poi_number = None, is_final_goal=False):
        id = str(uuid.uuid4())
        # the check is done in the add_vertex_with_id function
        if poi_number is not None:
            self.pois_set.add(poi_number)
        else :
            poi_number = None
        return self.add_vertex_with_id(id, posx, posy, poi_number, is_final_goal)

    def add_vertex_with_id(self, vertex_id, posx, posy, poi_number=None, is_final_goal=False):
        self.vertices[vertex_id] = Vertex(vertex_id, posx, posy, poi_number is not None, poi_number, is_final_goal)
        self.edges_from_vertex[vertex_id] = []
        if poi_number is not None:
            self.pois_set.add(poi_number)
        return True




    ### Edges functions
    def add_edge(self, start_id, end_id):
        id = str(uuid.uuid4())
        # the check is done in the add_edge_with_id function
        return self.add_edge_with_id(id, start_id, end_id)



    def get_vertices_list(self):
        return list(self.vertices.keys())


    def add_edge_with_incremental_id(self, start_id, end_id):
        id = "edge" + str(len(self.edges) + 1)
        return self.add_edge_with_id(id, start_id, end_id)




    def add_edge_with_id(self, edge_id, start_id, end_id):
        # check if the vertices exist
        if self.find_vertex_from_id(start_id) is None or self.find_vertex_from_id(end_id) is None:
            print("Vertex not found: ", start_id, end_id)
            return False
        # check if the edge already exists
        if edge_id in self.edges:
            print("Edge already exists: ", edge_id)
            return False
        # print("Adding edge with id: ", edge_id)
        edge_to_add = Edge(edge_id, start_id, end_id, self.find_vertex_from_id(start_id).get_posx(), self.find_vertex_from_id(start_id).get_posy(), self.find_vertex_from_id(end_id).get_posx(), self.find_vertex_from_id(end_id).get_posy())
        self.edges[edge_id] = edge_to_add
        if self.edges_from_vertex.get(start_id) is None:
            self.edges_from_vertex[start_id] = []
        self.edges_from_vertex[start_id].append(edge_to_add.get_end())
        return True




    def add_edge_with_id_and_positions(self, edge_id, start_posx, start_posy, end_posx, end_posy):
        start = self.find_vertex_from_position(start_posx, start_posy)
        end = self.find_vertex_from_position(end_posx, end_posy)
        return self.add_edge_with_id(edge_id, start.get_id(), end.get_id())

    def add_edge_from_positions(self, start_posx, start_posy, end_posx, end_posy):
        id = str(uuid.uuid4())
        return self.add_edge_with_id_and_positions(id, start_posx, start_posy, end_posx, end_posy)




    ### Goal vertices functions
    def set_vertex_as_goal_from_position(self, posx, posy):
        # check if the vertex exists
        vertex = self.find_vertex_from_position(posx, posy)
        if vertex is not None:
            self.goal_vertices.append(vertex)
            return True
        return False




    def is_goal_vertex(self, vertex_id):
        return vertex_id in self.goal_vertices




    def set_vertex_as_goal_from_id(self, vertex_id):
        # check if the vertex exists
        for vertex in self.vertices:
            if vertex.get_id() == vertex_id:
                self.goal_vertices.append(vertex)
                return True
        return False




    ## Functions for finding elements in the topological map
    def find_vertex_from_position(self, posx, posy):
        for vertex in self.vertices:
            if vertex.get_posx() == posx and vertex.get_posy() == posy:
                return vertex
        return None




    def find_vertex_from_id(self, vertex_id):
        # print(self.vertices)
        return self.vertices[vertex_id]


    def get_poi_vertices(self):
        poi_vertices = {}
        for vertex_id in self.vertices.keys():
            vertex = self.vertices[vertex_id]
            if vertex.is_poi():
                poi_vertices[vertex_id] = vertex
        return poi_vertices


    def find_edge_from_position(self, start, end):
        if start in self.edges_class_from_vertex.keys():
            if end in self.edges_class_from_vertex[start]:
                return self.edges_class_from_vertex[start][end]
        return None

    def find_edge_from_id(self, edge_id):
        return self.edges.get(edge_id, None)




    def get_vertex_distance(self, vertex1_name, vertex2_name):
        vertex1 = self.find_vertex_from_id(vertex1_name)
        vertex2 = self.find_vertex_from_id(vertex2_name)
        if vertex1 is not None and vertex2 is not None:
            return ((vertex1.get_posx() - vertex2.get_posx())**2 + (vertex1.get_posy() - vertex2.get_posy())**2)**0.5
        return None




    def get_all_edges_from_vertex(self, vertex_id):
        edges = []
        for edge in self.edges:
            if edge.get_start() == vertex_id or edge.get_end() == vertex_id:
                edges.append(edge)
        return edges


    ### Functions for saving and loading the topological map
    def save_topological_map(self, filename):
        with open(filename, 'w') as f:
            yaml.dump({'name': self.name, 
                       'vertices': [{'id': self.vertices[vertex_key].get_id(), 
                                     'posx': self.vertices[vertex_key].get_posx(), 
                                     'posy': self.vertices[vertex_key].get_posy(),
                                     'is_poi': self.vertices[vertex_key].is_poi(),
                                     'poi_number': self.vertices[vertex_key].get_poi_number(),
                                     'is_final_goal': self.vertices[vertex_key].is_final_goal()} for vertex_key in self.vertices.keys()], 
                        'edges': [{'id': self.edges[edge_key].get_id(), 
                                   'start': self.edges[edge_key].get_start(), 
                                   'end': self.edges[edge_key].get_end()} for edge_key in self.edges.keys()]}
                                   , f)



    def load_topological_map(self, filename):
        with open(filename, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self.name = data['name']
            self.vertices = {vertex['id']: Vertex(vertex['id'], vertex['posx'], vertex['posy'], vertex['is_poi'], vertex['poi_number'], vertex['is_final_goal']) for vertex in data['vertices']}
            for edge in data['edges']:
                start_vertex = self.find_vertex_from_id(edge['start'])
                end_vertex = self.find_vertex_from_id(edge['end'])
                self.add_edge_with_id(edge['id'], start_vertex.get_id(), end_vertex.get_id())
            self.compute_edges_from_vertex()
            self.compute_pois_set()
            self.compute_final_goal_vertices()
            # print("final_goal_vertices:", self.final_goal_vertices)
        




    def plot_topological_map(self, img_path, fig_size, fig_name, show_vertex_names = False):
        img = plt.imread(img_path)

        self.ax.set_xlim(fig_size[0], fig_size[1])
        self.ax.set_ylim(fig_size[2], fig_size[3])
        self.ax.set_aspect('equal')
        self.ax.set_title(fig_name)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        plt.imshow(img, cmap='gray', vmin=0, vmax=255, extent=fig_size)
        # plt.show()
        for edge_id in self.edges.keys():
            start = None
            end = None
            for vertex_id in self.vertices.keys():
                if vertex_id == self.edges[edge_id].get_start():
                    start = self.vertices[vertex_id]
                elif vertex_id == self.edges[edge_id].get_end():
                    end = self.vertices[vertex_id]
            if start is not None and end is not None:
                self.ax.plot([start.get_posx(), end.get_posx()], [start.get_posy(), end.get_posy()], 'pink')
                # if int(self.edges[edge_id].get_id()[4:]) > (len(self.edges) / 2):
                #     distance = ((start.get_posx() - end.get_posx())**2 + (start.get_posy() - end.get_posy())**2)**0.5
                #     self.ax.text(x = (start.get_posx() + end.get_posx()) / 2, y = (((start.get_posy() + end.get_posy()) / 2) - 0.4), s = self.edges[edge_id].get_id(), color = "red")
                #     self.ax.text(x = (start.get_posx() + end.get_posx()) / 2, y = (((start.get_posy() + end.get_posy()) / 2) - 0.8), s = str(round(distance, 2)), color = "black")
                # else:
                # self.ax.text(x = (start.get_posx() + end.get_posx()) / 2, y = (start.get_posy() + end.get_posy()) / 2,  s = self.edges[edge_id].get_id(), color = "blue")
                # plot the associated area colored in light blue
                area = self.edges[edge_id].get_area()
                # plot the vertices of the area in grey
                # if self.edges[edge_id].get_id() == "edge4":
                #     for v in area:
                #         self.ax.plot(v[0], v[1], 'go')
                # if area is not None:
                #     if int(self.edges[edge_id].get_id()[4:]) > len(self.edges) / 2:
                #         x = [area[0][0], area[1][0], area[2][0], area[3][0], area[0][0]]
                #         y = [area[0][1], area[1][1], area[2][1], area[3][1], area[0][1]]
                #         self.ax.fill(x, y, 'lightgreen')


        for vertex_id in self.vertices.keys():
            if self.vertices[vertex_id].is_poi():
                self.ax.plot(self.vertices[vertex_id].get_posx(), self.vertices[vertex_id].get_posy(), 'bo')
            else:
                self.ax.plot(self.vertices[vertex_id].get_posx(), self.vertices[vertex_id].get_posy(), 'ro')
            if show_vertex_names:
                self.ax.text(x = self.vertices[vertex_id].get_posx(), y = self.vertices[vertex_id].get_posy(), s = vertex_id + (f" (POI {self.vertices[vertex_id].get_poi_number()})" if self.vertices[vertex_id].is_poi() else ""), color = "blue")
            # plot a circle around the vertex to show the area in light green
            # circle  = plt.Circle((self.vertices[vertex_id].get_posx(), self.vertices[vertex_id].get_posy()), 1, color='lightgreen')
            # self.ax.add_artist(circle)
        # plt.show()


    def display_topological_map(self):
        plt.show()

    def save_figure(self, filename):
        # set high resolution
        self.fig.set_dpi(300)
        self.fig.tight_layout()
        # resolution  4k
        # self.fig.set_size_inches(3840/300, 3840/300)
        # remove white border
        self.fig.savefig(filename)


if __name__ == "__main__":
    topo_map = TopologicalMap()
    topo_map.load_topological_map(sys.argv[1])
    # topo_map.plot_topological_map("map.png", (0, 20, 0, 20), "Topological Map")
    if sys.argv[2] == "find_edge":
        print(topo_map.find_edge_from_position(sys.argv[3], sys.argv[4]).get_length())
    elif sys.argv[2] == "find_vertex":
        print(topo_map.find_vertex_from_id(sys.argv[3]))

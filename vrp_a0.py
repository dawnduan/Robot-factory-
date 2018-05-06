"""Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

###########################
# Problem Data Definition #
###########################
class DataProblem():
    """Stores the data for the problem"""
    def __init__(self, task_queue, robot_set):
        """Initializes the data for the problem"""
        self._num_vehicles = len(robot_set)
        self._locations = [t.location for t in task_queue]
        self._start_locations = [r.location for r in robot_set]
        self._end_locations = [0 for i in robot_set]
        self._loc_to_task = {t.location:i for i,t in enumerate(task_queue)}

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        """Gets locations"""
        return self._locations

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)
        
    @property
    def start_locations(self):
        """Gets start locations"""
        return self._start_locations
        
    @property
    def end_locations(self):
        """Gets end locations"""
        return self._end_locations
    
    @property
    def loc_to_task(self):
        """Gets end locations"""
        return self._loc_to_task
    

#######################
# Problem Constraints #
#######################
class CreateDistanceEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to return distance between points."""
    def __init__(self, distance_matrix, SPEED):
        """Initializes the distance matrix."""
        self._distances = {}
        self._speed = SPEED

        # translate distance_matrix to an obj
        for from_node in xrange(distance_matrix.shape[0]):
            self._distances[from_node] = {}
            for to_node in xrange(distance_matrix.shape[1]):
                    self._distances[from_node][to_node] = distance_matrix[from_node][to_node]/self._speed

    def distance_evaluator(self, from_node, to_node):
        """Returns the distance between the two nodes from the distance matrix"""
        return self._distances[from_node][to_node]
        
def add_distance_dimension(routing, distance_evaluator):
    # here distance_evaluator is dist_mtx
    """Add Global Span constraint"""
    distance = "Distance"
    maximum_distance = 1000000000000 # assume no limitation on robot travel distance
    routing.AddDimension(
        distance_evaluator,
        0, # null slack
        maximum_distance, # maximum distance per vehicle
        True, # start cumul to zero
        distance)
    distance_dimension = routing.GetDimensionOrDie(distance)
    # Try to minimize the max distance among vehicles.
    # /!\ It doesn't mean the standard deviation is minimized
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    # Returns the cost coefficient of the soft lower bound of a cumul variable for a given node. If no soft lower bound has been set, 0 is returned.

###########
# Printer #
###########
class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment, distance_matrix):
        """Initializes the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment

    @property
    def data(self):
        """Gets problem data"""
        return self._data

    @property
    def routing(self):
        """Gets routing model"""
        return self._routing

    @property
    def assignment(self):
        """Gets routing model"""
        return self._assignment
        
    @property
    def distance_matrix(self):
        """Gets distance matrix"""
        return self._distance_matrix
        
    @property
    def distance_matrix(self):
        """Gets distance matrix"""
        return self._distance_matrix

    def print(self):
        """Prints assignment on console"""
        # Inspect solution.
        total_dist = []; allocation = []
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for robot {0}:\n'.format(vehicle_id)
            route_dist = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                route_dist += distance_matrix[node_index][next_node_index]
                allocation.append([self.data.loc_to_task[node_index], vehicle_id])
                plan_output += ' {node_index} -> '.format(
                    node_index=node_index)
                index = self.assignment.Value(self.routing.NextVar(index))
                

            node_index = self.routing.IndexToNode(index)
            total_dist.append(route_dist)
            plan_output += ' {node_index}\n'.format(
                            node_index=node_index)
            plan_output += 'Time for the route {0}: {dist}\n'.format(
                vehicle_id,
                dist=route_dist)
            print(plan_output)
        print('Total Time for all routes: {dist}'.format(dist=max(total_dist)))
        return allocation
        
###
def algorithmZero(task_queue, robot_set, distance_matrix):
    """
    updated version from ortools, vehicle routing problem
    """
    SPEED = 1
    # Instantiate the data problem.
    d = DataProblem(task_queue, robot_set)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(d.num_locations, d.num_vehicles,
                                d.start_locations, d.end_locations)
    # Define weight of each edge
    distance_evaluator = CreateDistanceEvaluator(distance_matrix, SPEED).distance_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    add_distance_dimension(routing, distance_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    
    printer = ConsolePrinter(d, routing, assignment, distance_matrix)
    a = printer.print()
    return a

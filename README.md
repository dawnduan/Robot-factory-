# Robot-factory-

Goal: I'd like you to design two Python functions that assign a set of tasks to a set of available robots. The functions should take as arguments: i) the current task queue, ii) the current available robot set, and iii) a distance matrix between locations of interest in the warehouse (from 0 to n). The functions should return an allocation represented by a list of lists.

An example function structure is given as:

def algorithmOne(task_queue, robot_set, distance_matrix): 
   allocation = []
   < insert code here >
   return allocation

where task_queue is a list of task class objects, robot_set is a list of robot class objects, and distance matrix is a list of lists where distance_matrix[i][j] returns the distance between location of interest i and j. The 'allocation' list (your solution) should be a list of lists, where each list represents a task-robot pair. For example, allocation = [[0,0], [1,1]] would mean task 0 is allocated to robot 0 (0 -> 0) and task 1 is allocated to robot 1 (1 -> 1).

Each task in task_queue must be completed by the robot travelling from its current location to a location of interest. The location of interest required by the task can be accessed by task_queue[i].location, for task i, and the current location of the robot can be accessed by robot_set[i].location, for robot j.

Design two different assignment functions (they can be very simple!) and provide a sentence or two detailing which one you think would perform better if we wanted the robots to finish the tasks as quickly as possible.

from math import hypot
import numpy as np
from math import inf

def find_closest_rob(mtx, tasks, robots, cnt):
    # mtx r rows x t columns 
    # avai_tsk
    # return assignment in length of r
    #mtx = time_mtx; tasks = task_queue; robots = robot_set
    all_robs = range(0, len(robots))
    ROBOT = 0; TASK = 1 # index
    res = {}
    for i in all_robs:
        if cnt != 0:
            min_idx = np.unravel_index(np.argmin(mtx, axis=None), mtx.shape)
            t = min_idx[TASK]; r = min_idx[ROBOT]
            if t in res:
                print('duplicate task')
            else:
                res[t] = r 
                mtx[min_idx[ROBOT], :] = inf; mtx[:, min_idx[TASK]] = inf
                cnt -= 1
        else: # no task - completed!
            print('completed, no closest rob need to be found')
            break
    return res, mtx
    
def compute_time_mtx(tasks, robots, speed, distance_matrix):
    # tasks - lis, len = t
    # robots - lis, len = r
    # return mtx rxt where the elements is the time travel r -> 
    mtx = np.zeros( (len(robots), len(tasks)) )
    all_robots = range(0, len(robots))
    all_tasks = range(0, len(tasks))
    for r in all_robots:
        loc_r = robots[r].location
        for t in all_tasks:
            loc_t = tasks[t].location
            mtx[r][t] = distance_matrix[loc_r][loc_t]/speed
    return mtx
    
def algorithmOne(task_queue, robot_set, distance_matrix):   
# 
# Goal: I'd like you to design two Python functions that assign a set of tasks to a set of available robots. 
# 
# The functions should take as arguments: 
#     i) the current task queue: task_queue, a list of task class objects
#     ii) the current available robot set: robot_set, a list of robot class objects
#     iii) a distance matrix between locations of interest in the warehouse (from 0 to n).
#             distance_matrix, a list of lists 
#             where distance_matrix[i][j] returns the distance between location of interest i and j. 
# 
# The functions should return an allocation represented by a list of lists
#      where each list represents a task-robot pair. 
#      
# For example, allocation = [[0,0], [1,1]] would mean 
#     task 0 is allocated to robot 0 (0 -> 0) and 
#     task 1 is allocated to robot 1 (1 -> 1).
# 
# An example function structure is given as: 
    allocation = []; SPEED = 1 # robot travel speed
    all_robots = range(0, len(robot_set)); all_tasks = range(0, len(task_queue))
    avai_tsk = [i for i in all_tasks]
    time_mtx = compute_time_mtx(task_queue, robot_set, SPEED, distance_matrix)
    round = 0
    while len(avai_tsk) != 0:
        # avai_tsk keep tracks of rob assignment
        res, time_mtx = find_closest_rob(time_mtx, task_queue, robot_set, len(avai_tsk))
        for (t, r) in res.items():
            robot_set[r].location = task_queue[t].location
            allocation.append([t,r])
            avai_tsk.remove(t)
        for i in avai_tsk: # recalculating time mtx
            loc_t = task_queue[i].location
            for r in all_robots:
                loc_r = robot_set[r].location
                time_mtx[r][i] = distance_matrix[loc_r][loc_t]/SPEED
    return allocation
    
    
def test_compute_time_mtx():
    class vari:
        def __init__(self, loc):
            self.location = loc
    task_queue = [vari(i)for i in range(4)]
    robot_set = [vari(i) for i in range(4)]
    distance_matrix = [[3,2,1,4],
                        [6,5,3,1],
                        [2,5,6,4],
                        [2,3,5,7]]     
                        
    # Locations in block unit
    locations = \
            [(4, 4), # depot
                (2, 0), (8, 0), # row 0
                (0, 1), (1, 1),
                (5, 2), (7, 2),
                (3, 3), (6, 3),
                (5, 5), (8, 5),
                (1, 6), (2, 6),
                (3, 7), (6, 7),
                (0, 8), (7, 8)]
    # locations in meters using the city block dimension
                        
    a = algorithmOne(task_queue, robot_set, distance_matrix)
    print(len(set([i for [i,j] in a])) == 4)
    print(a)

test_compute_time_mtx()
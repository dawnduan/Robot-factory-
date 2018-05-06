from math import hypot
import numpy as np
from math import inf
##### HELPERS
def find_closest_rob(mtx, tasks, robots, cnt, output):
    # mtx r rows x t columns 
    # avai_tsk
    # return assignment in length of r
    #mtx = time_mtx; tasks = task_queue; robots = robot_set
    all_robs = range(0, len(robots))
    ROBOT = 0; TASK = 1 # index
    res = {}
    for i in all_robs:
        if cnt != 0:
            min_cost = np.argmin(mtx, axis=None)
            min_idx = np.unravel_index(min_cost, mtx.shape)
            t = min_idx[TASK]; r = min_idx[ROBOT]
            if t in res:
                print('duplicate task')
            else: #task allocation
                res[t] = r 
                output = process_printer(output, r, t, min_cost)
                mtx[min_idx[ROBOT], :] = inf; mtx[:, min_idx[TASK]] = inf
                cnt -= 1
        else: # no task - completed!
            print('completed, no closest rob need to be found')
            break
    return res, mtx, output
    
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
    
###### PRINTERS
def preface_printer(robot_set):
    o = []; tm = []
    for i,r in enumerate(robot_set):
        preface = 'Route for robot {0}:\n'.format(i)
        preface += 'Starting at' + ' {node_index} -> '.format(
                        node_index=r.location)
        o.append(preface); tm.append(0)
    o.append(tm)
    return o
    
def process_printer(o, robot, task, time_cost):
    ENDING = -1  # idx assignment
    o[robot] += 'Task' + ' {node_index} -> '.format(
                            node_index=task)
                            
    o[ENDING][robot] += time_cost
    return o
    
def back_to_0(robot_set, output, distance_matrix, SPEED):
    ENDING = -1  # idx assignment
    for i, robot in enumerate(robot_set):
        output[i] += 'Back to'+' {node_index}\n'.format(
                            node_index=0)
        output[ENDING][i] += distance_matrix[robot.location][0]/SPEED #update time cost for returning
    return output



###### MAIN
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
    allocation = []; SPEED = 1; ENDING = -1 # robot travel speed
    all_robots = range(0, len(robot_set)); all_tasks = range(0, len(task_queue))
    output = preface_printer(robot_set)
    # initialization of avai_task for track of task queue
    avai_tsk = [i for i in all_tasks]
    time_mtx = compute_time_mtx(task_queue, robot_set, SPEED, distance_matrix)
    # time_mtx as objective fcn (cost)
    while len(avai_tsk) != 0:
        # avai_tsk keep tracks of rob assignment
        res, time_mtx, output = find_closest_rob(time_mtx, task_queue, robot_set, len(avai_tsk), output)
        for (t, r) in res.items():
            robot_set[r].location = task_queue[t].location
            allocation.append([t,r])
            avai_tsk.remove(t)
        for i in avai_tsk: # recalculating time mtx
            loc_t = task_queue[i].location
            for r in all_robots:
                loc_r = robot_set[r].location
                time_mtx[r][i] = distance_matrix[loc_r][loc_t]/SPEED
                
    output = back_to_0(robot_set, output, distance_matrix, SPEED)
    # print the outcome
    total_dist = output[-1]
    output[-1] = 'Total time of all routes: {dist}'.format(dist=max(total_dist))
    [print(i + '\n') for i in output]

    return allocation
    

        

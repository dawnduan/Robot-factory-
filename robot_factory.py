# -*- coding: utf-8 -*-
"""
Created on Fri May  4 10:38:40 2018

@author: Dawn.Duan
"""

def main():
  # Creates the solver.
  solver = pywrapcp.Solver("simple_example")
# Creates the variables.
  num_vals = 3
  x = solver.IntVar(0, num_vals - 1, "x")
  y = solver.IntVar(0, num_vals - 1, "y")
  z = solver.IntVar(0, num_vals - 1, "z")
  solver.Add(x != y)
  db = solver.Phase([x, y, z], solver.CHOOSE_FIRST_UNBOUND, solver.ASSIGN_MIN_VALUE)
  
  solver.Solve(db)
  count = 0

  while solver.NextSolution():
    count += 1
    print("Solution", count, '\n')
    print("x = ", x.Value())
    print("y = ", y.Value())
    print("z = ", z.Value())
    print()
  print("Number of solutions:", count)
  
def algorithmOne(task_queue, robot_set, distance_matrix):   
'''  
Goal: I'd like you to design two Python functions that assign a set of tasks to a set of available robots. 

The functions should take as arguments: 
    i) the current task queue: task_queue, a list of task class objects
    ii) the current available robot set: robot_set, a list of robot class objects
    iii) a distance matrix between locations of interest in the warehouse (from 0 to n).
            distance_matrix, a list of lists 
            where distance_matrix[i][j] returns the distance between location of interest i and j. 

The functions should return an allocation represented by a list of lists
     where each list represents a task-robot pair. 
     
For example, allocation = [[0,0], [1,1]] would mean 
    task 0 is allocated to robot 0 (0 -> 0) and 
    task 1 is allocated to robot 1 (1 -> 1).

An example function structure is given as:
'''
   allocation = []
   < insert code here >
   return allocation
'''
Each task in task_queue must be completed by the robot travelling from its current location to a location of interest. 
The location of interest required by the task 
    can be accessed by task_queue[i].location, for task i, and 
    the current location of the robot can be accessed by robot_set[i].location, for robot j.

Design two different assignment functions (they can be very simple!) 
and provide a sentence or two detailing which one you think would perform better 
if we wanted the robots to finish the tasks as quickly as possible.
'''

def qa_my_fcn():
    task_queue_index = [0, 1, 2]
    robot_set_idx = [0, 1, 2]
    dist_mtx = [[2,4], [7,8], [5,0]]
    

def main():
  # Create the solver.
  solver = pywrapcp.Solver('jobshop')

  machines_count = 3
  jobs_count = 3
  all_machines = range(0, machines_count)
  all_jobs = range(0, jobs_count)
  # Define data.
  machines = [[0, 1, 2],
              [0, 2, 1],
              [1, 2]]

  processing_times = [[3, 2, 2],
                      [2, 1, 4],
                      [4, 3]]
  solver = pywrapcp.Solver('jobshop')
  
    # Creates jobs.
  all_tasks = {}
  for i in all_jobs:
    for j in range(0, len(machines[i])):
      all_tasks[(i, j)] = solver.FixedDurationIntervalVar(0,
                                                          horizon,
                                                          processing_times[i][j],
                                                          False,
                                                          'Job_%i_%i' % (i, j))

  # Creates sequence variables and add disjunctive constraints.
  all_sequences = []
  all_machines_jobs = []
  for i in all_machines:

    machines_jobs = []
    for j in all_jobs:
      for k in range(0, len(machines[j])):
        if machines[j][k] == i:
          machines_jobs.append(all_tasks[(j, k)])
    disj = solver.DisjunctiveConstraint(machines_jobs, 'machine %i' % i)
    all_sequences.append(disj.SequenceVar())
    solver.Add(disj)

  # Add conjunctive contraints.
  for i in all_jobs:
    for j in range(0, len(machines[i]) - 1):
      solver.Add(all_tasks[(i, j + 1)].StartsAfterEnd(all_tasks[(i, j)]))
  
  for i in all_jobs:
    for j in range(0, len(machines[i]) - 1):
      solver.Add(all_tasks[(i, j + 1)].StartsAfterEnd(all_tasks[(i, j)]))
    # Set the objective.
  obj_var = solver.Max([all_tasks[(i, len(machines[i])-1)].EndExpr()
                        for i in all_jobs])
      
    # Create search phases.
  sequence_phase = solver.Phase([all_sequences[i] for i in all_machines],
                                solver.SEQUENCE_DEFAULT)
  vars_phase = solver.Phase([obj_var],
                            solver.CHOOSE_FIRST_UNBOUND,
                            solver.ASSIGN_MIN_VALUE)
  main_phase = solver.Compose([sequence_phase, vars_phase])
    # Create the solution collector.
  collector = solver.LastSolutionCollector()

  # Add the interesting variables to the SolutionCollector.
  collector.Add(all_sequences)
  collector.AddObjective(obj_var)

  for i in all_machines:
    sequence = all_sequences[i];
    sequence_count = sequence.Size();
    for j in range(0, sequence_count):
      t = sequence.Interval(j)
      collector.Add(t.StartExpr().Var())
      collector.Add(t.EndExpr().Var())
      
    # Solve the problem.
  disp_col_width = 10
  if solver.Solve(main_phase, [objective_monitor, collector]):
    print("\nOptimal Schedule Length:", collector.ObjectiveValue(0), "\n")
    sol_line = ""
    sol_line_tasks = ""
    print("Optimal Schedule", "\n")

    for i in all_machines:
      seq = all_sequences[i]
      sol_line += "Machine " + str(i) + ": "
      sol_line_tasks += "Machine " + str(i) + ": "
      sequence = collector.ForwardSequence(0, seq)
      seq_size = len(sequence)

      for j in range(0, seq_size):
        t = seq.Interval(sequence[j]);
         # Add spaces to output to align columns.
        sol_line_tasks +=  t.Name() + " " * (disp_col_width - len(t.Name()))
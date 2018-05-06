from vrp import *
from fcn1 import *
import numpy as np
    
def main():
    class vari:
        def __init__(self, loc):
            self.location = loc
    task_queue = [vari(i)for i in range(4)]
    robot_set = [vari(0), vari(2)]
    distance_matrix = np.array([[0,2,1,4],
                                [2,0,3,1],
                                [1,3,0,5],
                                [4,1,5,0]]    ) 
    b = algorithmZero(task_queue, robot_set, distance_matrix)
    a = algorithmOne(task_queue, robot_set, distance_matrix)
    
    print(a, '\n', b)

if __name__ == '__main__':
  main()

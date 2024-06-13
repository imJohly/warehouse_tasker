import lkh
import numpy as np

## input for this node will end up being the static tf poses of the nodes.

class PoseProcessing:
 
    def __init__(self) -> None:
        pass

problem = lkh.LKHProblem()
problem.type = 'TSP'

problem.dimension = 8

problem.edge_weight_type = 'EXPLICIT'
problem.edge_weight_format = 'FULL_MATRIX'
problem.edge_weights = np.array([
        [0, 2, 100, 100, 100, 100, 100, 100],
        [2, 0, 1.41, 100, 100, 100, 100, 1.41],
        [100, 1.41, 0, 1, 100, 100, 100, 100],
        [100, 100, 1, 0, 1, 100, 100, 100],
        [100, 100, 100, 1, 0, 1, 100, 100],
        [100, 100, 100, 100, 1, 0, 1, 100],
        [100, 100, 100, 100, 100, 1, 0, 1],
        [100, 1.41, 100, 100, 100, 100, 1, 0]
    ])

# problem.demand_section = {
#         1 : [0],
#         2 : [0],
#         3 : [0],
#         4 : [0],
#         5 : [1],
#         6 : [1],
#         7 : [0],
#         8 : [0],
#     }

problem.fixed_edges = [[1, 2]]

if __name__ == '__main__':
    pass

import numpy as np

rotation_matrix = np.array([[0,-1, 0], [1, 0, 0], [0, 0, 1]])

x = np.array([3,2,0])

print(rotation_matrix @ x)

global_c = np.array([-2, 3, 0])

print(np.linalg.inv(rotation_matrix) @ global_c)
import pybullet as p
import numpy as np
from interpolation import linear_interpolation

if __name__ == "__main__":

    x1 = [0, 0, 0]
    x2 = [10, 0, 0]
    o1 = p.getQuaternionFromEuler([0, 0, 0])
    o2 = p.getQuaternionFromEuler([(-np.pi/2), 0, 0])
    path = linear_interpolation(x1, x2, 10, o1, o2)

    for position, orientation, _ in path:
        print(p.getEulerFromQuaternion(orientation))

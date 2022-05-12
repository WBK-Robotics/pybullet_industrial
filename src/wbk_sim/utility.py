import pybullet as p
import numpy as np

def draw_point(point, color=[0.0,1.0,0.0], length = 0.05,width=2.0):
    """Draws a point in the worldspace as a cross of 3 lines

    Args:
        pnt (array): 3 dimensional point
        color (list, optional): RGB color with alpha channel. Defaults to [0.0,1.0,0.0].
        length (float, optional): The length of the lines.  Defaults to 0.5.
        width (float, optional): The width of the lines. Defaults to 2.0.
    """
    
    direction = np.array([1,0,0])
    p.addUserDebugLine(point + length*direction,point - length*direction, 
                       lineColorRGB=color, lineWidth=width, lifeTime=0)

    direction = np.array([0,1,0])
    p.addUserDebugLine(point + length*direction,point - length*direction, 
                       lineColorRGB=color, lineWidth=width, lifeTime=0)

    direction = np.array([0,0,1])
    p.addUserDebugLine(point + length*direction,point - length*direction, 
                       lineColorRGB=color, lineWidth=width, lifeTime=0)
    

def draw_path(path,color=[0.0,1.0,0.0],width=2.0):
    path_steps =len(path[0])
    for i in range(1,path_steps):
        current_point = path[:,i]
        previous_point = path[:,i-1]
        p.addUserDebugLine(current_point,previous_point, 
                       lineColorRGB=color, lineWidth=width, lifeTime=0)
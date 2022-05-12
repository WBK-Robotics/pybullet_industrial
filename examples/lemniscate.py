import numpy as np

def build_lemniscate_path(midpoint,steps,height,length):
    """Function which builds a figure 8 path

    Args:
        midpoint ([type]): [description]
        steps ([type]): [description]
        height ([type]): [description]
        length ([type]): [description]

    Returns:
        [type]: [description]
    """
    path = np.zeros((3, steps))
    path[2,:]=height
    for i in range(steps):
        path_state = 1/steps*i*2*np.pi
        path[0,i]= length * np.cos(path_state)/(1+np.sin(path_state)**2)+midpoint[0]
        path[1,i]= length * np.sin(path_state) * np.cos(path_state)/(1+np.sin(path_state)**2)+midpoint[1]
    return path
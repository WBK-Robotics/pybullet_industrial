import pybullet as p
import pybullet_industrial as pi
import numpy as np
import time
import copy


def create_path(p1,p2,or1):
    path = pi.linear_interpolation(
                             np.array(p1), np.array(p2), 10)
    path.orientations = np.transpose(
                            [or1] * len(path.orientations[0]))
    return path

def move_along_path(endeffector: pi.EndeffectorTool, path: pi.ToolPath, stop=False):
    """Moving a designated endeffector along the provided path.
    Args:
        endeffector (pi.EndeffectorTool): Endeffector to be moved.
        path (pi.ToolPath): Array of points defining the path.
        stop (bool, optional): Whether or not to stop at the end of the movement.
    """
    for positions, orientations, tool_path in path:
        endeffector.set_tool_pose(positions, orientations)
        for _ in range(10):
            p.stepSimulation()
    if stop:
        for _ in range(100):
            p.stepSimulation()

def move_robot(robot: pi.RobotBase, path: pi.ToolPath, stop=False):
    """Moving a designated endeffector along the provided path.
    Args:
        endeffector (pi.EndeffectorTool): Endeffector to be moved.
        path (pi.ToolPath): Array of points defining the path.
        stop (bool, optional): Whether or not to stop at the end of the movement.
    """
    for positions, orientations, tool_path in path:
        robot.set_endeffector_pose(positions, orientations)
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.005)
    if stop:
        for _ in range(100):
            p.stepSimulation()      

class Gcode_class():


    def __init__(self, textfile: str= None, robot:pi.RobotBase= None, gripper:pi.Gripper = None):
        
        """The class for reading G-Code row by row from a textfile and to safe the processed Data in a Matrix.
        Every Letter of the G-Code has its own collum which are set NaN by default.
        After Reading the G-Code it can be played.
        
        Args:
            robot(pi.RobotBase)
            gripper(array of pi.Gripper)
        
        Returns:
            None
        """
       
        # self.robot = robot
        # self.gripper = gripper
        
        #Spawning the handed grippers in the enviroment
        if textfile is not None:
        
            
        
            if gripper is not None:
                self.gripper_spawn = []
                self.gripper_appr = []
                z = 0 
                for i in gripper:
                    spawn = i.get_tool_pose()[0]
                    spawn[2] = spawn[2] + 0.125 
                    appr = copy.deepcopy(spawn)
                    appr[2] = appr[2] + 1
                    self.gripper_spawn.append(spawn)
                    self.gripper_appr.append(appr)
                    z = z + 1
                
                #Simulation start with no active gripper  
                self.active_gripper = -1 #THIS NEEDS IMPROVEMENT --> CHECKING IF THERE IS A GRIPPER COUPLED
                input_array = self.read_gcode(textfile)
                self.run_gcode(input_array, robot, gripper)
            elif robot is not None:
                self.active_gripper = -1
                input_array = self.read_gcode(textfile)
                self.run_gcode(input_array, robot)
        

    def read_gcode(self,textfile:str):
        """Reads G-Code row by row and safes the processed Data in a Matric.
        Every Letter of the G-Code has its own collum which are set NaN by default.
        
        Args:
            tesxtfile(str): Source of information

        Returns:
            input_array(array)
        """
        #Creates the array with the infomation Variables that can be processed from the G-Code textfile
        #Array Design: ["Kommentar", 'G', 'M', 'T','X','Y','Z','A','B','C', "I", "J", "R", 'F']
        
        input_array = np.zeros(14, dtype='O')
        
        # Reads in the complete textfile and saves the information in the different collums of the input_array
        with open(textfile) as f:
            while True:
                line = f.readline()
                if not line: 
                    break
                
                new_line = np.zeros(14, dtype='O')
                new_line[:] = np.NaN
               
                
                if line[0] == "%":
                    new_line[0] = line
                    
                else:
                    new_line[0] = ""
                    data = line.split()
                    for i in data:
                        id = i[0]
                        
                        val2 = float(i[1:])  
                        
                        if id == "G" :
                            new_line[1] = int(val2)
                            
                        elif id =="M":
                            new_line[2] = int(val2)
                        elif id =="T":
                            new_line[3] = int(val2)
                        elif id == "X":
                            new_line[4] = val2
                        elif id =="Y":
                            new_line[5] = val2
                        elif id =="Z":
                            new_line[6] = val2
                        elif id == "A":
                            new_line[7] = val2
                        elif id =="B":
                            new_line[8] = val2
                        elif id =="C":
                            new_line[9] = val2
                        elif id =="I":
                            new_line[10] = val2
                        elif id =="J":
                            new_line[11] = val2
                        elif id =="R":
                            new_line[12] = val2
                        elif id =="F":
                            new_line[13] = val2    
                        else:
                            print(id, ": noch nicht implementiert")
                
              
                input_array = np.vstack((input_array,new_line))
       
        input_array = np.delete(input_array,0,0)
        return input_array

    def run_gcode(self, input_array, robot: pi.RobotBase, gripper: pi.Gripper = None):
        """Runs G-Code Row by Row. It is necessary to create the input_array with the readGcode(textfile) Method.
        
        Args:
           input_array(array)

        Returns:
            None
        """ 
        
        #Current Postion of the simulation in order enable path building
        last_point =  robot.get_endeffector_pose()[0] 
        last_or = robot.get_endeffector_pose()[1]
        last_or = p.getEulerFromQuaternion(last_or) #Conversion in order to adjust the orientation
        
        #Default Values for Running the G-Code
        plane = 2 #X-Y Plane for the circular interpolation -G2 & -G3
        offset = [0,0,0,0,0,0] #Offset for the zero offset -G54
       
        #Runs the infomation out of the input_array       
        for i in input_array:
            
            #Ignoring Comments
            comment = i[0]
            if comment != "":
                continue
            
            #Variables for using G-Code information
            g_com = i[1]
            m_com = i[2]
            t_com = i[3]
            f_val = i[7]
            r_val = i[12]

            #Checking system for the different commands

            #Checking for a G-command
            if not np.isnan(g_com): 
                
                #Checking for interpolation commands
                if g_com == 1 or g_com == 0 or g_com == 2 or g_com == 3: 
                    
                    #Setting the new point with the consideration of the offset
                    new_point = [0,0,0]
                    counter = 0
                    for n in i[4:7]: 
                        if np.isnan(n):
                            new_point[counter] = last_point[counter]
                        else:
                            new_point[counter] = n + offset[counter]
                        counter = counter + 1  
                    
                    #Setting the new orientation with the consideration of the offset
                    new_or = [0,0,0]
                    counter = 0
                    for n in i[7:10]:
                        if np.isnan(n):
                            new_or[counter] = last_or[counter]
                        else:
                            new_or[counter] = n + offset[counter+3]
                        counter = counter + 1 
                    orientation = p.getQuaternionFromEuler(new_or) 
                    
                    #Building the path if there is a linear interpolation
                    if g_com == 1 or g_com == 0:
                        path = pi.linear_interpolation(
                                np.array(last_point), np.array(new_point), 10)
                        path.orientations = np.transpose(
                                [orientation] * len(path.orientations[0]))
                    
                    #Building the path if there is a circular interpolation
                    elif g_com ==2:
                        path = pi.circular_interpolation( np.array(last_point), np.array(new_point),r_val,10,plane,True)
                    elif g_com ==3:
                        path = pi.circular_interpolation( np.array(last_point), np.array(new_point),r_val,10,plane,False)
                    if g_com == 2 or g_com == 3 :    
                        path.orientations = np.transpose(
                                [orientation] * len(path.orientations[0]))

                    #Moving the Gripper if activated
                    if not self.active_gripper == -1:
                        move_along_path(gripper[self.active_gripper], path)
                        last_point = new_point
                        last_or = new_or
                    
                    #Moving the Robot if activated
                    else:
                        move_robot(robot, path)
                        last_point = new_point
                        last_or = new_or
              

    	        #Activation of the zero offset
                elif g_com ==54:
                    offset = last_point + last_or
                
                #Deactivation of the zero offset
                elif g_com ==500:
                    offset = [0,0,0,0,0,0]
                
                #Plane selelection for circular interpolation 
                elif g_com == 17:
                    plane = 2 #X-Y Plane
                elif g_com == 18:
                    plane = 1 #X-Z Plane
                elif g_com == 19:
                    plane = 0 #Y-Z Plane
            
            #Checking for a M-command        
            elif not np.isnan(m_com):
                
                #Close Gripper
                if m_com==10:
                    gripper[self.active_gripper].actuate(1.0)
                    for _ in range(25):
                        p.stepSimulation()
                        time.sleep(0.01)
                
                #Open Grippper
                elif m_com==11:
                    gripper[self.active_gripper].actuate(0.0)
                    for _ in range(25):
                        p.stepSimulation()
                        time.sleep(0.01)
            
            #Checking for a T-command
            elif not np.isnan(t_com):
                
                #Running a tool change logic
                or1 = p.getQuaternionFromEuler([-np.pi/2, 0, 0]) 
                
                #Checking for Decoupling
                if t_com == 0 and not self.active_gripper == -1:
                    
                    #Move in
                    path = create_path(last_point, self.gripper_appr[self.active_gripper], or1)
                    move_robot(robot,path)                    
                    last_point = self.gripper_appr[self.active_gripper]

                    #Move to span
                    path = create_path(last_point, self.gripper_spawn[self.active_gripper], or1)
                    move_robot(robot,path)
                    last_point = self.gripper_spawn[self.active_gripper]

                    #Decouple
                    gripper[self.active_gripper].decouple()
                    
                    #Move out
                    path = create_path(last_point, self.gripper_appr[self.active_gripper], or1)
                    move_robot(robot,path)                    
                    last_point = self.gripper_appr[self.active_gripper]
                    self.active_gripper = -1

                #Checking for Decoupling   
                else:
                    
                    #Checking if same Gripper is already used
                    if not self.active_gripper == (t_com - 1):
                        
                        #Move to approach
                        self.active_gripper = t_com - 1
                        path = create_path(last_point, self.gripper_appr[self.active_gripper], or1)
                        move_robot(robot,path)                    
                        last_point = self.gripper_appr[self.active_gripper]
                        
                        #Move to spawn-point
                        path = create_path(last_point, self.gripper_spawn[self.active_gripper], or1)
                        move_robot(robot,path)
                        last_point = self.gripper_spawn[self.active_gripper]                    
                        
                        #Couple
                        gripper[self.active_gripper].couple(robot, 'link6')
                        gripper[self.active_gripper].actuate(0.0)
                
                        #Move out
                        path = create_path(last_point, self.gripper_appr[self.active_gripper], or1)
                        move_robot(robot,path)                    
                        last_point = self.gripper_appr[self.active_gripper]
                    
                        


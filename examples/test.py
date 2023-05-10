import os
import pybullet as p
from pybullet_industrial import linear_interpolation
from pybullet_industrial import ToolPath


class Processor:
    def __init__(self, gcode):
        self.gcode = gcode
        self.g500 = False

    def __iter__(self):
        self.current_index = 0
        return self

    def __len__(self):
        return len(self.gcode)

    def __next__(self):
        if self.current_index <= len(self)-1:
            i = self.current_index
            self.current_index += 1
            if self.gcode[i] == "G0":
                return self.gcode[i]
            if self.gcode[i] == "G500":
                self.g500 = True
            if self.gcode[i] == "G1":
                commands = "G0"
                self.gcode.insert(i+1, commands)
                print("commands inserted")

        else:
            raise StopIteration


def read_gcode(filename: str):
    """Reads G-Code row by row and saves the processed Data in
    a List.
    Comments that start with % are ignored and all the other data is
    stored as it gets read in.
    Every Line in the G-Code resembles the same structure as the text file

    Args:
        filename (str): Source of information

    Returns:
        gcode
    """
    with open(filename, encoding='utf-8') as f:
        gcode = []

        # Loop over the lines of the file
        for line in f.readlines():

            # Initialize a new line as a list
            new_line = []

            # Read in G-Code if line is not a comment and not empty
            if line[0] != "%" and len(line) > 1:

                # Split the line into its components
                data = line.split()

                # Loop over the components
                for i in data:
                    # Determine the ID of the component
                    id_val = i[0]

                    # Extract the value of the component
                    val2 = float(i[1:])

                    if id_val in ["G", "M", "T"]:
                        # Insert the value into the corresponding
                        # column of the new line
                        new_line.append([id_val, int(val2)])
                    else:
                        new_line.append([id_val, val2])

                # Add the finished line to the list
                gcode.append(new_line)

        return gcode


if __name__ == "__main__":

    p1 = [0, 0, 0]
    p2 = [1, 0, 0]
    path = linear_interpolation(p1, p2, 10)

    dirname = os.path.dirname(__file__)
    textfile = os.path.join(dirname, 'Gcodes', 'test.txt')
    gcode = read_gcode(textfile)
    print(gcode)
    insert_postion = 2
    for pos, ori, _ in path:
        ori = p.getEulerFromQuaternion(ori)
        array = [['G', 0], ['X', pos[0]], ['Y', pos[1]], [
            'Z', pos[2]], ['A', ori[0]], ['B', ori[1]], ['C', ori[2]]]
        gcode.insert(insert_postion, array)
        insert_postion += 1

    print(gcode)

    # gccode = ["G0", "G500", "G1", "G0"]

    # gcode_object = Processor(gccode)

    # for gcode in gcode_object:
    #     if gcode is None:
    #         print("operation with no simualtion")
    #     else:
    #         for _ in range(1):
    #             print(gcode)
    #             for _ in range(1):
    #                 print("simulating")

from toolpath import ToolPath
import numpy as np

if __name__ == "__main__":

    # 1. Define initial positions (3xN array)
    positions = np.array([
        [1, 2, 3],  # X-coordinates
        [1, 2, 3],  # Y-coordinates
        [1, 2, 3]   # Z-coordinates
    ])

    # 2. Define orientations (4xN array for quaternions)
    orientations = np.array([
        [1, 2, 3],  # Quaternion X
        [0, 0, 0],  # Quaternion Y
        [0, 0, 0],  # Quaternion Z
        [0, 0, 0]   # Quaternion W
    ])

    # 3. Define tool activations (1xN array of booleans)
    tool_activations = np.zeros(len(positions[0]))  # Tool active at first and third positions

    # Create the ToolPath object
    tool_path = ToolPath(positions, orientations)

    # 4. Access the starting pose
    start_position, start_orientation = tool_path.get_start_pose()
    print("Start Position:", start_position)
    print("Start Orientation:", start_orientation)

    for position, orientation, tool_activation in tool_path:
        print("Position:", position)
        print("Orientation:", orientation)
        print("Tool Activation:", tool_activation)
        print("---")

    # Create an iterator from the ToolPath
    iterator = iter(tool_path)

    print("Iterating through the toolpath:")
    # Use the iterator with next()
    while True:
        try:
            position, orientation, tool_activation = next(iterator)
            print("Position:", position)
            print("Orientation:", orientation)
            print("Tool Activation:", tool_activation)
            print("---")
        except StopIteration:
            # StopIteration is raised when the iterator is exhausted
            print("End of toolpath.")
            break
    print("end of code")

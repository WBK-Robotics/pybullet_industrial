import numpy as np
import matplotlib.pyplot as plt
import random

# Simple class to represent a node in the tree


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# Euclidean distance between two nodes


def distance(node1, node2):
    return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

# Steer towards a target node with a maximum step size


def steer(from_node, to_node, step_size=0.1):
    dist = distance(from_node, to_node)
    if dist <= step_size:
        return to_node
    theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + step_size * np.cos(theta)
    new_y = from_node.y + step_size * np.sin(theta)
    return Node(new_x, new_y)

# Check if a point is inside an obstacle (circular obstacle for simplicity)


def is_in_obstacle(node, obstacles):
    for (ox, oy, radius) in obstacles:
        if distance(node, Node(ox, oy)) <= radius:
            return True
    return False

# Check if the line between two nodes is collision-free


def is_collision_free(from_node, to_node, obstacles, step_size=0.01):
    dist = distance(from_node, to_node)
    steps = int(dist / step_size)
    for i in range(steps):
        interp_x = from_node.x + (to_node.x - from_node.x) * i / steps
        interp_y = from_node.y + (to_node.y - from_node.y) * i / steps
        if is_in_obstacle(Node(interp_x, interp_y), obstacles):
            return False
    return True

# RRT algorithm with collision detection


def rrt(start, goal, obstacles, max_iters=10000, step_size=0.1):
    nodes = [start]
    for _ in range(max_iters):
        # Randomly sample a point in the space
        rand_node = Node(random.random(), random.random())

        # Find the nearest node in the tree
        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))

        # Steer from the nearest node towards the random node
        new_node = steer(nearest_node, rand_node, step_size)

        # Check if the new node is collision-free
        if is_collision_free(nearest_node, new_node, obstacles):
            new_node.parent = nearest_node
            nodes.append(new_node)

            # Check if we've reached the goal
            if distance(new_node, goal) <= step_size and is_collision_free(new_node, goal, obstacles):
                goal.parent = new_node
                nodes.append(goal)
                print("Goal reached!")
                return nodes, goal  # Return the tree and the goal node

    return nodes, None

# Trace back the path from the goal to the start


def get_final_path(goal_node):
    path = []
    current_node = goal_node
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent
    return path[::-1]  # Return reversed path from start to goal

# Plot the RRT, obstacles, and the final path


def plot_rrt(nodes, final_path, start, goal, obstacles):
    plt.figure()
    plt.grid(True)

    # Plot obstacles
    for (ox, oy, radius) in obstacles:
        obstacle_circle = plt.Circle((ox, oy), radius, color='gray', fill=True)
        plt.gca().add_patch(obstacle_circle)

    # Plot all nodes and connections (the entire tree)
    for node in nodes:
        if node.parent is not None:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-')

    # Highlight the final path
    for i in range(len(final_path) - 1):
        plt.plot([final_path[i].x, final_path[i + 1].x],
                 [final_path[i].y, final_path[i + 1].y], 'r-', linewidth=2)

    # Highlight start and goal
    plt.plot(start.x, start.y, 'go', label="Start")  # Start is green
    plt.plot(goal.x, goal.y, 'ro', label="Goal")    # Goal is red
    plt.legend()
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.title("RRT Path with Collision Detection")
    plt.show()


# Main execution
if __name__ == "__main__":
    start_node = Node(0, 0)
    goal_node = Node(1, 1)

    # Define some circular obstacles (center_x, center_y, radius)
    obstacles = [
        (0.5, 0.5, 0.1),  # Obstacle in the middle
        (0.7, 0.2, 0.1),  # Another obstacle
    ]

    # Run RRT
    rrt_tree, goal_reached = rrt(
        start_node, goal_node, obstacles, step_size=0.1)

    # If the goal was reached, trace and plot the final path
    if goal_reached:
        final_path = get_final_path(goal_reached)
        plot_rrt(rrt_tree, final_path, start_node, goal_node, obstacles)
    else:
        print("Goal not reached within the iteration limit.")

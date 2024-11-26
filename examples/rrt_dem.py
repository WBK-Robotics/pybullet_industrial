import ompl.base as ob
import ompl.geometric as og
import matplotlib.pyplot as plt

# Define the state space
class PlanningProblem:
    def __init__(self):
        # Define a 2D state space (x, y in [0, 1])
        self.space = ob.RealVectorStateSpace(2)
        self.bounds = ob.RealVectorBounds(2)
        self.bounds.setLow(0)
        self.bounds.setHigh(1)
        self.space.setBounds(self.bounds)
        # Set up the space information
        self.si = ob.SpaceInformation(self.space)
    def is_valid(self, state):
        """Define obstacle checking"""
        obstacles = [
            (0.5, 0.5, 0.1),  # Obstacle in the middle
            (0.7, 0.2, 0.1),  # Another obstacle
        ]
        for ox, oy, radius in obstacles:
            dist = ((state[0] - ox)**2 + (state[1] - oy)**2)**0.5
            if dist <= radius:
                return False
        return True

    def plan(self, start_coords, goal_coords, use_rrt_star=False):
        # Set the validity checker
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_valid))

        # Define the problem
        start = ob.State(self.space)
        goal = ob.State(self.space)
        start[0], start[1] = start_coords
        goal[0], goal[1] = goal_coords
        problem = ob.ProblemDefinition(self.si)
        problem.setStartAndGoalStates(start, goal)

        # Set the planner
        if use_rrt_star:
            planner = og.RRTstar(self.si)  # Use RRT*
        else:
            planner = og.RRT(self.si)  # Use RRT
        planner.setProblemDefinition(problem)
        planner.setup()

        # Solve the problem
        if planner.solve(1.0):  # 1.0 seconds timeout
            return problem.getSolutionPath()
        else:
            return None

# Plotting the results
def plot_solution(path, obstacles, title, filename):
    plt.figure()
    plt.grid(True)

    # Plot obstacles
    for ox, oy, radius in obstacles:
        obstacle_circle = plt.Circle((ox, oy), radius, color='gray', fill=True)
        plt.gca().add_patch(obstacle_circle)

    # Plot the solution path
    states = path.getStates()
    x_coords = [state[0] for state in states]
    y_coords = [state[1] for state in states]
    plt.plot(x_coords, y_coords, 'r-', linewidth=2, label='Path')

    # Highlight start and goal
    plt.plot(x_coords[0], y_coords[0], 'go', label="Start")
    plt.plot(x_coords[-1], y_coords[-1], 'ro', label="Goal")
    plt.legend()
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.title(title)
    plt.savefig(filename)
    plt.close()


# Main execution
if __name__ == "__main__":
    problem = PlanningProblem()

    # Obstacles
    obstacles = [
        (0.5, 0.5, 0.1),  # Obstacle in the middle
        (0.7, 0.2, 0.1),  # Another obstacle
    ]

    # Plan with RRT
    rrt_solution = problem.plan((0.0, 0.0), (1.0, 1.0), use_rrt_star=False)
    if rrt_solution:
        print("Path found with RRT!")
        plot_solution(rrt_solution, obstacles, "RRT Path", "examples/plots/rrt_path.png")
    else:
        print("No path found with RRT.")

    # Plan with RRT*
    rrt_star_solution = problem.plan((0.0, 0.0), (1.0, 1.0), use_rrt_star=True)
    if rrt_star_solution:
        print("Path found with RRT*!")
        plot_solution(rrt_star_solution, obstacles, "RRT* Path", "examples/plots/rrt_star_path.png")
    else:
        print("No path found with RRT*.")

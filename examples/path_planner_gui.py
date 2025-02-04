import tkinter as tk
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np
import sys

JOINT_INCREMENT = 0.1


class PathPlannerGUI:
    """
    GUI for the Path Planner, allowing users to control joints, update obstacles,
    and execute paths. Now shows three status indicators:
      - Collision status (red/green)
      - End-effector upright status (red/green)
      - Clearance cost (numeric value)
    """

    def __init__(self, root, robot: pi.RobotBase, path_planner: pi.PathPlanner,
                 collision_checker: pi.CollisionChecker, obstacle):
        """
        Initializes the PathPlanner GUI.

        Args:
            root (tk.Tk): The root Tkinter window.
            robot: The robot instance from PyBullet Industrial.
            path_planner: The PathPlanner instance for motion planning.
            collision_checker: The CollisionChecker instance for collision management.
            obstacle: The obstacle to manipulate.
        """
        self.root = root
        self.robot = robot
        self.path_planner = path_planner
        self.collision_checker = collision_checker
        self.obstacle = obstacle
        self.start = self.robot.get_joint_state()  # Start configuration
        self.goal = self.robot.get_joint_state()  # Goal configuration
        self.joint_order = self.robot.get_moveable_joints()[0]
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")
        self.joint_values = [tk.StringVar(value="0")
                             for _ in range(len(self.joint_order))]
        self.obstacle_values = [tk.StringVar(value="0") for _ in range(6)]
        # Status indicator variables for collision and end-effector status.
        self.collision_status = tk.StringVar(value="green")
        self.endeffector_status = tk.StringVar(value="green")
        self.clearance_cost_text = tk.StringVar(value="0.00")
        self.current_box_size = [0.5, 0.5, 0.05]
        self.create_widgets()
        self.set_allowed_collision()
        self.update_joint_positions()
        self.set_initial_obstacle_values()
        self.update_status()

    def create_widgets(self):
        """
        Creates and arranges the widgets in the GUI.
        """
        # Create the layout for the robot's joints
        for i, joint_name in enumerate(self.joint_order):
            tk.Label(self.root, text=f"{joint_name}").grid(
                row=i, column=0, padx=5, pady=5)
            tk.Button(self.root, text="+",
                      command=lambda i=i: self.increment_joint(i)).grid(
                row=i, column=1, padx=5, pady=5)
            tk.Button(self.root, text="-",
                      command=lambda i=i: self.decrement_joint(i)).grid(
                row=i, column=2, padx=5, pady=5)
            tk.Entry(self.root, textvariable=self.joint_values[i],
                     width=10).grid(row=i, column=3, padx=5, pady=5)

        # Create the layout for the obstacle's position and orientation
        obstacle_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obstacle_labels):
            tk.Label(self.root, text=f"{label}").grid(
                row=i, column=4, padx=5, pady=5)
            tk.Button(self.root, text="+",
                      command=lambda i=i: self.increment_obstacle(i)).grid(
                row=i, column=5, padx=5, pady=5)
            tk.Button(self.root, text="-",
                      command=lambda i=i: self.decrement_obstacle(i)).grid(
                row=i, column=6, padx=5, pady=5)
            tk.Entry(self.root, textvariable=self.obstacle_values[i],
                     width=10).grid(row=i, column=7, padx=5, pady=5)

        # Status Indicators
        status_row = len(self.joint_order)
        # Collision Status Indicator
        tk.Label(self.root, text="Collision Status:").grid(
            row=status_row, column=0, padx=5, pady=10)
        self.collision_light = tk.Label(self.root, bg=self.collision_status.get(),
                                        width=10, height=2)
        self.collision_light.grid(row=status_row, column=1, padx=5, pady=10)

        # End-effector Status Indicator
        tk.Label(self.root, text="End-effector Status:").grid(
            row=status_row, column=2, padx=5, pady=10)
        self.endeffector_light = tk.Label(self.root, bg=self.endeffector_status.get(),
                                          width=10, height=2)
        self.endeffector_light.grid(row=status_row, column=3, padx=5, pady=10)

        # Clearance Cost Indicator
        tk.Label(self.root, text="Clearance Cost:").grid(
            row=status_row, column=4, padx=5, pady=10)
        self.clearance_cost_label = tk.Label(self.root, textvariable=self.clearance_cost_text,
                                             width=10, height=2)
        self.clearance_cost_label.grid(row=status_row, column=5, padx=5, pady=10)

        # Control Buttons
        control_row = status_row + 1
        tk.Button(self.root, text="Set as Start",
                  command=self.set_as_start).grid(row=control_row,
                                                  column=0, pady=10)
        tk.Button(self.root, text="Set as End",
                  command=self.set_as_goal).grid(row=control_row,
                                                  column=1, pady=10)
        tk.Button(self.root, text="Set Allowed Collision",
                  command=self.set_allowed_collision).grid(row=control_row,
                                                            column=2, pady=10)
        tk.Button(self.root, text="Plan and Execute",
                  command=self.plan_and_execute).grid(row=control_row,
                                                       column=3, pady=10)
        tk.Button(self.root, text="Update Joint Positions",
                  command=self.update_joint_positions).grid(row=control_row+1,
                                                              column=0,
                                                              columnspan=2,
                                                              pady=10)
        # Obstacle Size Buttons
        tk.Button(self.root, text="Shrink Obstacle",
                  command=self.shrink_obstacle).grid(row=control_row+2,
                                                     column=4, pady=10)
        tk.Button(self.root, text="Grow Obstacle",
                  command=self.grow_obstacle).grid(row=control_row+2,
                                                   column=5, pady=10)
        # End Buttons
        tk.Button(self.root, text="Check Endeffector",
                  command=self.check_endeffector_upright).grid(row=control_row+2,
                                                              column=6, pady=20)
        tk.Button(self.root, text="Upright Endeffector",
                  command=self.make_endeffector_upright).grid(row=control_row+2,
                                                              column=7, pady=20)
        tk.Button(self.root, text="Exit", command=self.root.quit).grid(
            row=control_row+3, column=0, columnspan=4, pady=20)

    def update_status(self):
        """
        Updates all status indicators: collision status, end-effector
        orientation, and clearance cost.
        """
        # Update collision status light
        if self.path_planner.is_state_valid(None):
            self.collision_status.set("green")
        else:
            self.collision_status.set("red")
        self.collision_light.config(bg=self.collision_status.get())

        # Update end-effector status light (using ValidityChecker method)
        if self.path_planner.validity_checker.check_endeffector_upright():
            self.endeffector_status.set("green")
        else:
            self.endeffector_status.set("red")
        self.endeffector_light.config(bg=self.endeffector_status.get())

        # Update clearance cost label based on the clearance value.
        clearance = self.collision_checker.get_max_distance()
        # Clearance cost is computed as 1/(clearance + epsilon)
        cost = 1 / (clearance + sys.float_info.min)
        self.clearance_cost_text.set(f"{cost:.2f}")

    def make_endeffector_upright(self):
        position = self.robot.get_endeffector_pose()[0]
        orientation = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
        self.robot.set_endeffector_pose(position, orientation)
        self.update_joint_positions()
        self.update_status()

    def check_endeffector_upright(self):
        """
        Checks if the end effector is upright.
        """
        orientation = p.getEulerFromQuaternion(self.robot.get_endeffector_pose()[1])
        target_orientation = np.array([-np.pi / 2, 0, 0])
        tolerance = np.array([0.3, 0.3, 2 * np.pi])
        if np.all(np.abs(orientation - target_orientation) <= tolerance):
            print("End-effector is upright.")
        else:
            print("End-effector is not upright.")
        self.update_status()

    def shrink_obstacle(self):
        current_position, current_orientation = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [(extent - (extent * 0.2)) for extent in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
                                                            halfExtents=self.current_box_size),
            basePosition=current_position,
            baseOrientation=current_orientation
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def grow_obstacle(self):
        current_position, current_orientation = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [(extent + (extent * 0.2)) for extent in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
                                                            halfExtents=self.current_box_size),
            basePosition=current_position,
            baseOrientation=current_orientation
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def set_initial_obstacle_values(self):
        position, orientation_quat = p.getBasePositionAndOrientation(self.obstacle)
        orientation_euler = p.getEulerFromQuaternion(orientation_quat)
        for i in range(3):
            self.obstacle_values[i].set(f"{position[i]:.2f}")
        for i in range(3, 6):
            self.obstacle_values[i].set(f"{orientation_euler[i - 3]:.2f}")

    def update_joint_positions(self):
        joint_state = self.robot.get_joint_state()
        for i, joint_name in enumerate(self.joint_order):
            position = joint_state[joint_name]['position']
            self.joint_values[i].set(f"{position:.2f}")
        self.update_status()

    def increment_joint(self, index):
        current_value = float(self.joint_values[index].get())
        new_value = current_value + JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_value <= self.joint_limits[1][joint_name]:
            self.joint_values[index].set(f"{new_value:.2f}")
            self.apply_joint_position(index, new_value)

    def decrement_joint(self, index):
        current_value = float(self.joint_values[index].get())
        new_value = current_value - JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_value >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_value:.2f}")
            self.apply_joint_position(index, new_value)

    def increment_obstacle(self, index):
        current_value = float(self.obstacle_values[index].get())
        new_value = current_value + 0.1
        self.obstacle_values[index].set(f"{new_value:.2f}")
        self.update_obstacle()

    def decrement_obstacle(self, index):
        current_value = float(self.obstacle_values[index].get())
        new_value = current_value - 0.1
        self.obstacle_values[index].set(f"{new_value:.2f}")
        self.update_obstacle()

    def update_obstacle(self):
        position = [float(self.obstacle_values[i].get()) for i in range(3)]
        orientation_euler = [float(self.obstacle_values[i].get()) for i in range(3, 6)]
        orientation_quat = p.getQuaternionFromEuler(orientation_euler)
        p.resetBasePositionAndOrientation(self.obstacle, position, orientation_quat)
        self.update_status()

    def apply_joint_position(self, index, position):
        joint_name = self.joint_order[index]
        target = {joint_name: position}
        self.robot.reset_joint_position(target)
        self.update_status()

    def set_as_start(self):
        self.start = {joint_name: float(self.joint_values[i].get())
                      for i, joint_name in enumerate(self.joint_order)}
        print("Start configuration set:", self.start)

    def set_as_goal(self):
        self.goal = {joint_name: float(self.joint_values[i].get())
                     for i, joint_name in enumerate(self.joint_order)}
        print("Goal configuration set:", self.goal)

    def set_allowed_collision(self):
        self.collision_checker.allow_collision_links = \
            self.collision_checker.get_collision_links()
        self.collision_checker.update_collision_settings()
        print("Allowed collisions set.")
        self.update_status()

    def plan_and_execute(self):
        print("Planning and executing path...")
        res, joint_path = self.path_planner.plan_start_goal(self.start, self.goal)
        if res:
            for joint_configuration, tool_activation in joint_path:
                self.robot.reset_joint_position(joint_configuration, True)
                time.sleep(0.01)
            self.update_joint_positions()
            print("Path execution completed.")
        else:
            print("No solution found.")
        self.update_status()

import tkinter as tk
import time
import pybullet_industrial as pi
import pybullet as p
import numpy as np
import sys

JOINT_INCREMENT = 0.1

class PathPlannerGUI:
    """
    GUI for the Path Planner, allowing users to control joints, update
    obstacles, and execute paths. Shows collision status and constraint status.
    """

    def __init__(self, root, robot: pi.RobotBase,
                 path_planner: pi.PathPlanner,
                 collision_checker_list: list, obstacle, constraint_functions):
        """
        Initializes the PathPlanner GUI.

        Args:
            root (tk.Tk): The root Tkinter window.
            robot: The robot instance from PyBullet Industrial.
            path_planner: The PathPlanner instance for motion planning.
            collision_checker_list (list): List of CollisionChecker objects for collision management.
            obstacle: The obstacle to manipulate.
            constraint_functions (list): List of functions (no-arg lambdas) that return a boolean
                                         indicating if a constraint is satisfied.
        """
        self.root = root
        self.robot = robot
        self.path_planner = path_planner
        self.collision_checker_list = collision_checker_list
        self.obstacle = obstacle
        self.constraint_functions = constraint_functions
        self.start = self.robot.get_joint_state()  # Start config.
        self.goal = self.robot.get_joint_state()   # Goal config.
        self.joint_order = self.robot.get_moveable_joints()[0]
        self.joint_limits = self.robot.get_joint_limits()
        self.root.title("PyBullet Path Planning")
        self.joint_values = [tk.StringVar(value="0")
                             for _ in range(len(self.joint_order))]
        self.obstacle_values = [tk.StringVar(value="0")
                                for _ in range(6)]
        # Status indicator for collision.
        self.collision_status = tk.StringVar(value="green")
        # Status indicator for constraint functions.
        self.constraint_status = tk.StringVar(value="green")
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
        # Layout for the robot's joints.
        for i, joint_name in enumerate(self.joint_order):
            tk.Label(self.root, text=f"{joint_name}").grid(
                row=i, column=0, padx=5, pady=5)
            tk.Button(self.root, text="+",
                      command=lambda i=i: self.increment_joint(i)
                      ).grid(row=i, column=1, padx=5, pady=5)
            tk.Button(self.root, text="-",
                      command=lambda i=i: self.decrement_joint(i)
                      ).grid(row=i, column=2, padx=5, pady=5)
            tk.Entry(self.root, textvariable=self.joint_values[i],
                     width=10).grid(row=i, column=3, padx=5, pady=5)

        # Layout for the obstacle's position and orientation.
        obstacle_labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(obstacle_labels):
            tk.Label(self.root, text=f"{label}").grid(
                row=i, column=4, padx=5, pady=5)
            tk.Button(self.root, text="+",
                      command=lambda i=i: self.increment_obstacle(i)
                      ).grid(row=i, column=5, padx=5, pady=5)
            tk.Button(self.root, text="-",
                      command=lambda i=i: self.decrement_obstacle(i)
                      ).grid(row=i, column=6, padx=5, pady=5)
            tk.Entry(self.root, textvariable=self.obstacle_values[i],
                     width=10).grid(row=i, column=7, padx=5, pady=5)

        # Status Indicator for collision.
        status_row = len(self.joint_order)
        tk.Label(self.root, text="Collision Status:").grid(
            row=status_row, column=0, padx=5, pady=10)
        self.collision_light = tk.Label(self.root,
                                        bg=self.collision_status.get(),
                                        width=10, height=2)
        self.collision_light.grid(row=status_row, column=1,
                                  padx=5, pady=10)
        # Status Indicator for constraints.
        tk.Label(self.root, text="Constraint Status:").grid(
            row=status_row, column=2, padx=5, pady=10)
        self.constraint_light = tk.Label(self.root,
                                         bg=self.constraint_status.get(),
                                         width=10, height=2)
        self.constraint_light.grid(row=status_row, column=3,
                                   padx=5, pady=10)

        # Control Buttons.
        control_row = status_row + 1
        tk.Button(self.root, text="Set as Start",
                  command=self.set_as_start).grid(
                      row=control_row, column=0, pady=10)
        tk.Button(self.root, text="Set as End",
                  command=self.set_as_goal).grid(
                      row=control_row, column=1, pady=10)
        tk.Button(self.root, text="Set Allowed Collision",
                  command=self.set_allowed_collision).grid(
                      row=control_row, column=2, pady=10)
        tk.Button(self.root, text="Plan and Execute",
                  command=self.plan_and_execute).grid(
                      row=control_row, column=3, pady=10)
        # Obstacle Size Buttons.
        tk.Button(self.root, text="Shrink Obstacle",
                  command=self.shrink_obstacle).grid(
                      row=control_row+1, column=4, pady=10)
        tk.Button(self.root, text="Grow Obstacle",
                  command=self.grow_obstacle).grid(
                      row=control_row+1, column=5, pady=10)
        # Exit Button.
        tk.Button(self.root, text="Exit",
                  command=self.root.quit).grid(
                      row=control_row+2, column=0, columnspan=4,
                      pady=20)

    def update_status(self):
        """
        Updates the collision status and constraint status indicators.
        """
        # Update collision status.
        valid_collision = all(cc.check_collision() for cc in self.collision_checker_list)
        self.collision_status.set("green" if valid_collision else "red")
        self.collision_light.config(bg=self.collision_status.get())
        # Update constraint status.
        self.update_constraint_status()

    def update_constraint_status(self):
        """
        Checks all constraint functions. If all return True, the status is green;
        otherwise, it's red.
        """
        valid_constraints = all(fn() for fn in self.constraint_functions)
        self.constraint_status.set("green" if valid_constraints else "red")
        self.constraint_light.config(bg=self.constraint_status.get())

    def shrink_obstacle(self):
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [extent - (extent * 0.2)
                                 for extent in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def grow_obstacle(self):
        pos, orn = p.getBasePositionAndOrientation(self.obstacle)
        self.current_box_size = [extent + (extent * 0.2)
                                 for extent in self.current_box_size]
        p.removeBody(self.obstacle)
        self.obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.current_box_size),
            basePosition=pos, baseOrientation=orn
        )
        self.set_initial_obstacle_values()
        self.update_status()

    def set_initial_obstacle_values(self):
        pos, orn_q = p.getBasePositionAndOrientation(self.obstacle)
        orn_e = p.getEulerFromQuaternion(orn_q)
        for i in range(3):
            self.obstacle_values[i].set(f"{pos[i]:.2f}")
        for i in range(3, 6):
            self.obstacle_values[i].set(f"{orn_e[i-3]:.2f}")

    def update_joint_positions(self):
        joint_state = self.robot.get_joint_state()
        for i, jn in enumerate(self.joint_order):
            pos = joint_state[jn]['position']
            self.joint_values[i].set(f"{pos:.2f}")
        self.update_status()

    def increment_joint(self, index):
        cur = float(self.joint_values[index].get())
        new_val = cur + JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_val <= self.joint_limits[1][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.apply_joint_position(index, new_val)

    def decrement_joint(self, index):
        cur = float(self.joint_values[index].get())
        new_val = cur - JOINT_INCREMENT
        joint_name = self.joint_order[index]
        if new_val >= self.joint_limits[0][joint_name]:
            self.joint_values[index].set(f"{new_val:.2f}")
            self.apply_joint_position(index, new_val)

    def increment_obstacle(self, index):
        cur = float(self.obstacle_values[index].get())
        new_val = cur + 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def decrement_obstacle(self, index):
        cur = float(self.obstacle_values[index].get())
        new_val = cur - 0.1
        self.obstacle_values[index].set(f"{new_val:.2f}")
        self.update_obstacle()

    def update_obstacle(self):
        pos = [float(self.obstacle_values[i].get()) for i in range(3)]
        orn_e = [float(self.obstacle_values[i].get())
                for i in range(3, 6)]
        orn_q = p.getQuaternionFromEuler(orn_e)
        p.resetBasePositionAndOrientation(self.obstacle, pos, orn_q)
        self.update_status()

    def apply_joint_position(self, index, position):
        joint_name = self.joint_order[index]
        target = {joint_name: position}
        self.robot.reset_joint_position(target)
        self.update_status()

    def set_as_start(self):
        self.start = {jn: float(self.joint_values[i].get())
                      for i, jn in enumerate(self.joint_order)}
        print("Start configuration set:", self.start)

    def set_as_goal(self):
        self.goal = {jn: float(self.joint_values[i].get())
                     for i, jn in enumerate(self.joint_order)}
        print("Goal configuration set:", self.goal)

    def set_allowed_collision(self):
        for cc in self.collision_checker_list:
            cc.set_safe_state()
        print("Allowed collisions set.")
        self.update_status()

    def plan_and_execute(self):
        print("Planning and executing path...")
        res, joint_path = self.path_planner.plan_start_goal(
            self.start, self.goal)
        if res:
            for joint_conf, tool_act in joint_path:
                self.robot.reset_joint_position(joint_conf, True)
                time.sleep(0.01)
            self.update_joint_positions()
            print("Path execution completed.")
        else:
            print("No solution found.")
        self.update_status()

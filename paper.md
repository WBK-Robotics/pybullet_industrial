---
title: 'PyBullet Industrial: A process-aware robot simulation'
tags:
  - Python
  - robotics
  - simulation
  - milling
  - force calculation
  - 3D printing
  - production processes
authors:
  - name: Jan Baumgärtner
    orcid: 0000-0002-7825-3476
    affiliation: 1
  - name: Malte Hansjosten
    affiliation: 1
  - name: Dominik Schönhofen
    affiliation: 2
  - name: Prof. Dr.-Ing. Jürgen Fleischer
    orcid: 0000-0003-0961-7675
    affiliation: 1
affiliations:
  - name: wbk Institute of Production Science
    index: 1
  - name: Independent Researcher
    index: 2
date: 22 September 2022
bibliography: paper.bib
---

# Summary
The trend towards individualized products and the increasing demand for a greater number of variants require a rethinking in the production engineering environment. In the context of this transformation, we see robots taking on more and more manufacturing tasks [@wsk].
The development of this field is hampered by a toolchain gap: While there are a large number of robot simulations and process simulations there is not yet a simple simulation environment that combines the two and allows the user to investigate the interplay of both.
Process simulation in this case refers to the simulation of manufacturing processes which according to [@process_sim] is defined as the use of one or
more physical mechanisms to transform the shape and/or form of a workpiece.
While process simulation is a vast field that studies different levels of detail, we focus on the simulation of how the process affects the environment and the robot.

To meet this challenge we developed PyBullet Industrial. This Python package extends the open-source multi-body physics package PyBullet with manufacturing process models to simulate manufacturing applications that add material, remove material or simply move material.
A sample of concrete manufacturing applications in each category can be seen in Figure \ref{manu_process}.

The package not only simulates the environmental effect of the processes but also the forces imparted on the robot. It also allows the dynamic switching of processes with the same robot corresponding to tool changes during the manufacturing process. The package also contains utility functions such as path builder classes which are based on G-code (also called RS274) interpolation schemes [@g-code] or a variety of drawing and visualization functions.



# Statement of need
PyBullet Industrial was developed for the interdisciplinary field of robot manufacturing.
While there are a large number of simulation tools for robotics research such as Gazebo [@gazebo], CoppeliaSim [@coppeliasim], or webots [@webots], their capabilities all end at the robot end effector as they are unable to simulate manufacturing processes.
In the same vein, there are several popular FE simulation tools such as Abaqus [@abaqus] capable of simulating process behavior.
These simulations end at the tool as they are not meant to simulate the systems that move the tool.
Since robots are now performing more and more manufacturing tasks studying and accounting for the interaction between robots and processes becomes ever more important.
This requires a simulation that can simulate robots and processes.

PyBullet Industrial closes this gap by taking classical robot multibody simulations and extending them using simple process simulations which impact the environment.
PyBullet Industrial is thus the first process-aware robot simulation platform built for research.
Note that PyBullet Industrial neither aims to develop perfect process simulations nor robot simulations, it focuses on the interplay of both.
Example applications of PyBullet Industrial are:

* Designing joint controllers that compensate for the large process forces during milling
* Designing path planning algorithms for 3D printing that can detect if a robot combines with a previously printed object.
* Checking the coating of an object in complex scenarios where the object is moved by a robot while another one is spraying paint.


# Overview

Robot simulations typically start at the base and stop at the end effector while process simulations typically start at the process and end where the tool is connected to the machine. PyBullet Industrial divides functionality similarly by employing a `RobotBase` class simulating the multibody dynamics of a Robot manipulator and an `EndeffectorTool` class capable of simulating processes.
A sample simulation view with both objects can be seen in Figure \ref{PyBullet_industrial_overview}.

![Overview over the two main Objects \label{PyBullet_industrial_overview}](PyBullet_industrial_overview.png)

These objects can be deployed into a standard PyBullet simulation environment and used to build manufacturing scenarios.

## Robot objects

The `RobotBase` class builds upon PyBullets URDF (Universal Robot Description Format) [@urdf] import feature which allows the loading of dynamic multibody robot models. The class adds several convenient interfaces which allow the setting and measuring of joint and end effector states. This latter allows the user to reposition the end effector without worrying about the underlying kinematics.
The list of interfaces includes:

- A joint state interface that allows the user to set and read joint positions, velocities, and torques.
- A end effector state interface that allows the user to set and read the end effector position, orientation, and velocity. The inverse kinematics is in this case calculated using PyBullets built-in inverse kinematics solver.
- A world state interface that allows the user to set and read the world position and orientation of the robot base.



## End effector Tools

End effector tools are the main novelty of this library and implement various process models.
An `EndeffectorTool` object can be coupled with a robot attaching it to the flange of the end effector.
The tool provides a positioning interface that automatically calls the end effector interface of a coupled robot making it easy to reposition the tool center point in space.

Note that coupling and decoupling of tools can be done during runtime to simulate tool quick changes common in complex manufacturing cells.
The geometry of a specific tools is defined by a URDF file where the last link is the default tool center point although this can be changed by the user.

While the base object implements the main interfaces and structure of the class, different process models are implemented as children of the `EndeffectorTool` object.
These models can be grouped into three different categories according to how they interact with material as seen in Figure \ref{manu_process}.

![Classes of Manufacturing processes that can be simulated using this package \label{manu_process}](manufacturing_processes.png)

The adding of material is done using the `Extruder` class which uses raycasts [@raycast] to spawn objects either on the surface of another object or at the end of the raycast.
This is indicated in Figure \ref{extruder}, which also shows the extruder parameters that can be set.

![Extruder parameters visualization \label{extruder}](extruder.png)

These objects are implemented as Materials that can have different properties from massless particles sticking to surfaces (such as paint) to physical bodies like 3D printing plastic.
By default, no force is imparted during such processes although custom force models can be added by implementing the `calculate_process_force` function.

Removing of material is either done using the `MillingTool` which uses the Kienzle force model [@kienzle] for planar milling or the `Remover` which is the twin of the `Extruder` and can be used to simulate ablative processes such as sandblasting or waterjet cutting.
The process force model for milling can be seen in Figure \ref{kienzle_force}. Here the chip thickness exponent and the material-specific force are material-dependent.

![Cutting Force calculation as described by the Kienzle Model [@kienzle] \label{kienzle_force}](cutting_force.png)

The moving of material is achieved using grippers. PyBullet Industrial supports both finger grippers and suction grippers for this purpose.

For camera-based applications, the library also contains a camera sensor tool that can be used to simulate process inspection tasks.

## Utility
To make development easier, the library has several utility functions.
This includes the `ToolPath` class which has a custom iterator making it easy for tools and robots to follow predetermined paths. These paths can be built using different interpolation functions such as linear interpolation, spline interpolation, or circular interpolation.
Path positions and orientations can be visualized using drawing functions as seen in Figure \ref{robot_path}.
These underlying functions can also be used to visualize arbitrary coordinate systems or robot link poses.

![Sample visualization of a Toolpath \label{robot_path}](robot_paths.png)



# Conclusion:
PyBullet Industrial is a novel simulation platform for robot manufacturing research.
It allows the simulation of robots and processes in a single environment.
While the library offers the basic functionality to simulate robots and processes, these blocks need to be parameterized and combined to create a meaningful simulation.
Future work will focus on the development of such parameterization and combination methods.
For deployment on real robots, the library will also be extended to directly parse g-code files and convert them into tool paths.
For direct control, a ROS interface will be added to allow the use of ROS controllers.



# Example
A simple example highlighting how the library can be used can be seen in the following code snippet.

```python

import os

import pybullet as p
import pybullet_data
import pybullet_industrial as pi

dirname = os.path.dirname(__file__)
urdf_file1 = os.path.join(dirname,
                          'robot_descriptions', 'comau_nj290_robot.urdf')
urdf_file2 = os.path.join(dirname,
                          'robot_descriptions', '3d_printing_head.urdf')

physics_client = p.connect(p.GUI)
p.setPhysicsEngineParameter(numSolverIterations=5000)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("cube.urdf", [1.9, 0, 0.5], useFixedBase=True)

robot = pi.RobotBase(urdf_file1, [0, 0, 0], [0,0,0,1])

extruder_properties = {'maximum distance': 0.5,'opening angle': 0,
                       'material': pi.Plastic,'number of rays': 1}
extruder = pi.Extruder(
    urdf_file2, [1.9, 0, 1.2], [0,0,0,1], extruder_properties)
extruder.couple(robot, 'printing_coupling_frame')

test_path = pi.build_box_path(
    [1.9, 0, 1.03], [0.5, 0.6], 0.1, [0, 0, 0, 1], 100)

extruder.set_tool_pose(*test_path.get_start_pose())
for _ in range(50):
    p.stepSimulation()

test_path.draw()
for positions, orientations, _ in test_path:
    extruder.set_tool_pose(positions, p.getQuaternionFromEuler([0, 0, 0]))
    particle = extruder.extrude()

    p.stepSimulation()

```

In this simple example, a `RobotBase` is coupled to an `Extruder` tool and a `ToolPath` path is built to print a box with rounded corners.
The path is iterated using the built-in iterator and the extruder is moved along the path extruding particles.
More examples can be found in the [examples folder](https://github.com/WBK-Robotics/PyBullet_industrial/tree/main/examples) of the repository.


# References
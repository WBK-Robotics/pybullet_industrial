---
title: 'Pybullet Industrial: A python package for simulating industrial processes and robots'
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
  - name: Malte (last name?)
    orcid:
  - name: Dominik Schönhofen
  - name: Prof. Dr.-Ing. Jürgen Fleischer
    orcid: 0000-0003-0961-7675
affiliations:
  - name: wbk Institute of Production Science
    index: 1
  - name: Karlsruhe Institute for Technology
    index: 2
date: 22 September 2022
bibliography: paper.bib
---

# Summary
The trend towards individualized products and the increasing demand for a greater number of variants require a rethinking in the production engineering environment. In the context of this transformation we see robots taking on more and more manufacturing tasks.
The development of this field is hampered by a toolchain gap: While there are a large number of robot simultions and process simulations there is not yet a simple simulation environment that combines the two and allows the user to investigate the interplay of both.

Pybullet Industrial extends the open source library Pybullet with several process models to simulate manufacturing applications that add material, remove material or simply move material. A sample of concrete manufacturing applications in each category can be seen down below:

%TODO add list of example applications which accompanying pybullet industrial screenshots

The package not only simulates the environmental effect of the processes but also the forces emparted on the robot. It also allows the dynamic switching of processes with the same robot corresponding to tool changes during the manufacturing process. The package also contains utility functionality such as path builder classes which are based on G-code interpolation schemes or a variety of drawing and visualisation functions.



# Statement of need
Pybullet industrial was developed for the inderdiscplinary field of robot manufacturing.
While there are a large number of simulation tools for robotics reserch such as Gazebo, CoppeliaSim or webots, their capabilties all end at the robot endeffector as they are unable to simulate manufacturing processes.
In the same veine there are several popular FE simulation tools capable of simulating process behavior.
These however end at the tool as they are not meant to simulate the sytems that actually move the tool.

Pybullet_industrial closes this gap by taking classical multibody simulations that end at the endeffector and then deploying simple process simulations which impact the environment.
Pybullet_industrial is thus the first process aware robot simulation plattform build for research.
Note that pybullet industrial neither aims to develop perfect process simulations nor robot simulations, it focusses on the interplay of both.
Example applications could be:

* Designing joint controllers that compensate for the large process forces during milling
* Design path planning algorithms for 3D printing that can detect if a robot combines with a previously printed object.
* Check the coating of an object in  complex scenarios where the object is moved by a robot while another one is spraying paint.


# Overview

Robot simulations typically end at the endeffector while process simulations typically end where the tool is connected to the machine. Pybullet industrial divides functionality similarly by employing a Robot class simulating the multibody dynamics of a Robot manipulator and an Endeffector Tool class capable of simulating processes.

These objects can be deployed into a standart pybullet simulation environment and used to build manufacturing scenarios.

## Robot objects

The RobotBase class build upon pybullets urdf import feature which allows the loading of dynamic multibody robot models. The class adds a number of convenient interfaces which allow the setting and meaturing of joint and endeffector states. This latter allows the user to reposition the robots endeffector without worrying about the underlying kinematics.

## Endeffector Tools

Endeffector Tools are the main novelty of this library and implement various process models.
An EndeffectorTool object can be coupled with a robot attaching it at the flange of the endeffector.
The tool provides its own positioning interface wich automatically calls the endeffector interface of a coupled robot making it easy to reposition the tool center point in space.

Note that coupling and decoupling of tools can be done during runtime to simulate tool quickchanges common in complex manufacturing cells.

While the base object implements the main interfaces and structure of the class, different proccess models are implemented as children of the EndeffectorTool object. These proccess models can be grouped into three different categories:

% Todo hier bild der 3 kategorien hin

The adding of material is done using an extruder which uses raycasts to spawn objects either on the surface of another objects or at the end of the raycast.
These objects are implemented as Materials which can have different properties from masslessly sticking to surfaces (such as paint) to physical bodies like 3D printing plastic.
By default no force is emparted during such processes though custom force models can be added by implementing the calculate_process_force function.

Removing of material is either done using the MillingTool which uses the kienzle force model for planar milling or the Remover which is the twin of the remover and can be used to simulate procceses such as sandblasting or waterjet cutting.

% Add image of kienzle model and properly reference it

The moving of material is achieved using grippers. Pybullet industrial supports both finger grippers and suction grippers for this purpose.

For camera based applications the library also contains a camera sensor tool that can be used to simulate process inspection tasks.

## Utility
To make development easier, the library has a number of utility functionality.
This includes the ToolPath class which its own custom iterator making it easy for tools and robots to follow predetermined paths. These paths can be build using different interpolation functions such as linear interpolation, spline interpolation or circular interpolation.
Path positions and orientations can be visualized using drawing functions.
These underlying functions can also be used to visualize arbitrary coordinate system or robot link poses.

% TODO add toopath image

# References

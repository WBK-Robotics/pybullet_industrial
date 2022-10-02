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

%TODO add image of three categories

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



# References

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

#Summary
The trend towards individualized products and the increasing demand for a greater number of variants require a rethinking in the production engineering environment. In the context of this transformation we see robots taking on more and more manufacturing tasks.
The development of this field is hampered by a toolchain gap: While there are a large number of robot simultions and process simulations there is not yet a simple simulation environment that combines the two and allows the user to investigate the interplay of both.

Pybullet Industrial extends the open source library Pybullet with several process models to simulate manufacturing applications that add material, remove material or simply move material. A sample of concrete manufacturing applications in each category can be seen down below:

%TODO add image of three categories

The package not only simulates the environmental effect of the processes but also the forces emparted on the robot. It also allows the dynamic switching of processes with the same robot corresponding to tool changes during the manufacturing process. The package also contains utility functionality such as path builder classes which are based on G-code interpolation schemes or a variety of drawing and visualisation functions.



#Statement of need
Pybullet industrial was developed in order to model both the production processes and the robot behaviour.
The currently available software on the market does not offer both aspects in an easy to use fashion while being open source.
Some popular products like Gazebo, CoppeliaSim or Webots can model some aspects of production lines, e.g. a robot gripping an object, but lack the simulation of milling or 3D-printing.
Machining processes are usually simulated with FE software.
The problem with this approach is, that it is not comprehensive enough.
Its level of detail is very focussed and does not include the greater frame of the robot or the production line.
Softwares that covers both, like Siemens NX is usually not open source.
Pybullet industrial fulfills this demand while being available for free and easy to use.


#References

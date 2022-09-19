#################
Using the Package
#################

This page describes the general design of the classes and functions in this package to enable users to use the package to its full potential.  The package is designed to be used in a variety of ways, and the following sections describe the different ways in which the package can be used.
The aim of pybullet_industrial was to merge the worlds of process simulations and multibody robot simulations by providing a combined solution with reasonable performance in both domains.
The main robot simulation functionality is provided by a dedicated Robot object called RobotBase, while the process is simulation by a clas called EndeffectorTool.
Plainly speaking this means that robot tools encapsulate the various manufacturing processes.

The following sections will dive deeper into the robot and endeffector objects and details how they can be used to simulate manufacturing scenarios.

.. image:: images/robot_tool_overview.svg
    :width: 60%
    :align: center
    :alt: robot_tool_overview

*************
Robot objects
*************

Robot objects are on of the main objects in the package. There main purpose is to load a dynamic robot simulation into a pybullet simulation and provide a set of functions to control the robot and get information about the robot state.
The pybullet_industrial package provides a class called RobotBase that can be used to load a robot from a urdf (universal robot description file) file and to interact with it.
A robot in this case meaning a robot manipulator, that is to say a stationary robot with a fixed base and a number of joints that can be actuated.

Joint interfaces
================

pybullet_industrial provides interfaces for setting and measuring the state of these joints. The state of a single joint is a dictionary containing the following keys:

- position: the current position of the joint (in radians for revolute and in meters for prismatic joints)
- velocity: the current velocity of the joint (in radians per second for revolute and in meters per second for prismatic joints)
- reaction force: the current reaction force of the joint (in Newtons)
- torque: the current effort of the joint (in Newtons for revolute and in Newtons per meter for prismatic joints)

Endeffector interfaces
======================

In industrial robotics one often times does not care for the joint state of the robot, but rather for the state of the endeffector. 

.. warning::
    The endeffector is the part of the robot that is attached to the end of the last joint and that is typically used to interact with the environment. 
    But for the pybullet_industrial package interaction with the environment is handled by the EndeffectorTool class. This means that the endeffector refers in this case to the end of the robots flange.

The pybullet_industrial package provides interfaces for setting and measuring the state of the endeffector. 
These intertfaces make it possible to set the desired position and orientation of the endeffector and to measure the current position and orientation of the endeffector.
Note that providing the orientation is optional, in this case the robot assumes a arbitrary rotation at a given position.

.. important::
    The orientation is given as a quaternion, which is a 4-tuple of floats. 
    The first three elements of the tuple are the imaginary part of the quaternion and the last element is the real part.


Utility functionality
=====================

Apart from the joint and endeffector interfaces the pybullet_industrial package provides some utility functionality.
These enable resetting the robots state and moving the robot to a new position.
More information about these functions can be found in the Code documentation.

*****************
Endeffector tools
*****************

The endeffector tool is the main object for simulating processes in the pybullet_industrial package.
In robotic manfucatuing theses processes can be grouped in three categories:

- Adding Material (Such as welding, gluing, 3d printing etc.)
- Removing Material (Such as milling, drilling, cutting etc.)
- Moving Material (Such as moving a part from one place to another)

Each of theses proceses types is supported by a dedicated subclass. 
The Base class still provides a lot of functionality common between all three types.

Adding material
===============

Removing material
=================

Moving material
===============
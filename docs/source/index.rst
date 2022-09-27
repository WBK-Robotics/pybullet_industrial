.. pybullet industrial documentation master file, created by
   sphinx-quickstart on Tue May 17 13:03:53 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.




Welcome to pybullet industrial's documentation!
===============================================

.. image:: logo.png
    :width: 80%
    :align: center
    :alt: pybullet_industrial_logo

The Pybullet_industrial package combines the world of classical robot simulations with the world of industrial processes.
The library is capable of simulating different manufacturing tools and workpieces, as well as the robot itself.
The simulation is based on the pybullet library.
With the help of the pybullet_industrial package you will be able to:

- simulate additive manufacturing processes
- simulate milling processes and how the resulting forces impact the robot
- simulating paint coating scenarios
- simulate the handling of complex tasks using a variety of grippers


What pybullet_industrial is not
-------------------------------

While the library can simulate process forces, it is no substitute for a proper FEM simulation.
This is because the library does not focus on modeling a given process, but rather on the robot's interaction with the process.

Also, take note that the library does not claim to bridge the sim2real gap. It is thus not meant to be used for training robots.
This would require much more sophisticated models, which are not the focus of this library. However, using the tools of this package it is certainly possible to develop such models.

.. toctree::

   getting_started
   how_to_use
   tutorials
   code_docu


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

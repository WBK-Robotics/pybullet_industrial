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

Pybullet_industrial is a process-aware robot simulation.
It aims to enable scientists and researchers to easily simulate robotics scenarios where a robot is participating in a manufacturing process.
It achieves this by combining the world of classical robot simulations with the world of industrial processes.
The library is capable of simulating different manufacturing tools and workpieces, as well as the robot itself.
With the help of the pybullet_industrial package you will be able to:

- simulate robot based additive manufacturing processes
- simulate milling processes and how the resulting forces impact a robot
- simulating paint coating scenarios
- simulate the handling of complex tasks using a variety of grippers


What pybullet_industrial is not
-------------------------------

The library is not meant as a proper FEM simulation tool or virtual commisioning tool.
Instead its tools and techniques focus on the development of robot control applications for manufacturing scenarios.
As such the process models are implemented to be performant but lightweight. This means that a sim2real gap is to be expected.
Due to the flexibility of the library, it is however possible to implement more complex process models for such purposes.

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

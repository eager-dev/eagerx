******
EAGERx
******
.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
   :target: https://github.com/psf/black
   :alt: codestyle

.. image:: https://readthedocs.org/projects/mushroomrl/badge/?version=latest
   :target: https://eagerx.readthedocs.io/en/latest/?badge=latest
   :alt: Documentation Status

.. image:: https://github.com/eager-dev/eagerx/actions/workflows/ci.yml/badge.svg?branch=master
   :target: https://github.com/MushroomRL/mushroom-rl/actions/workflows/continuous_integration.yml
   :alt: Continuous Integration

.. image:: https://api.codeclimate.com/v1/badges/3146dce3dd4c3537834c/maintainability
   :target: https://codeclimate.com/github/eager-dev/eagerx/maintainability
   :alt: Maintainability

.. image:: https://api.codeclimate.com/v1/badges/3146dce3dd4c3537834c/test_coverage
   :target: https://codeclimate.com/github/eager-dev/eagerx/test_coverage
   :alt: Test Coverage

**Streamlining the transfer of simulated robot learning to the real-world.**

.. contents::
   :depth: 2


What is EAGERx
==============
EAGERx enables users to easily define new tasks, switch from one sensor to another,
and switch from simulation to reality with a single line of code by being invariant to the physics engine.
EAGERx explicitly addresses the differences in learning between simulation and reality,
with essential features for roboticists such as a safety layer, signal delay simulation, and controller switching for resets.
A single RL pipeline that works with both the simulated and real robots eliminates the chance for mismatches between the simulation and reality implementation.
The defined task follows the OpenAI Gym interface, so one can plug in algorithms from established RL libraries
(e.g., `Stable-baselines3 <https://github.com/DLR-RM/stable-baselines3>`_ ) to solve the task afterward, again minimizing implementation errors.

`Full documentation and tutorials available here <https://eagerx.readthedocs.io/en/latest/>`_.

Example
=================
ToDo:

Installation
============

You can do a minimal installation of ``EAGERx`` with:

.. code:: shell

    pip3 install eagerx

EAGERx depends on a minimal ROS installation. Fortunately, you **can** use eagerx anywhere as you would any python package,
so it does **not** impose a ROS package structure on your project.
See `here <ROS_>`_ for installation instruction.

Extras: GUI
---------------------

To install the whole set of features, you will need additional packages.
You can install everything by running:

.. code:: shell

    pip3 install eagerx-gui

ToDo: add example and gif of GUI

Extras: training visualization
---------------------

For training visualization, either a ``desktop`` or ``desktop-full`` ROS installation is required.
See `here <ROS_>`_ for installation instruction.

ToDo: add example and gif of visualization.

Dependencies
============
Below you find instructions for installing dependencies required for EAGERx (and the whole set of features).

ROS
---------------------

See the `ROS Installation Options <https://eagerx.readthedocs.io/en/latest/>`_, or do the following.
By replacing ``<DISTRO>`` with the supported ROS distributions (``noetic``, ``melodic``),
and ``<PACKAGE>`` with the installation type (``ros-base``, ``desktop``, ``desktop-full``),
a minimal ros installation can be installed with:

.. code:: shell

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-<DISTRO>-<PACKAGE>
    sudo apt-get install ros-<DISTRO>>-cv-bridge

Make sure to source ``/opt/ros/<DISTRO>/setup.bash`` in the environment you use ``eagerx`` in.
It can be convenient to automatically source this script every time a new shell is launched.
These commands will do that for you if you:

.. code:: shell

      echo "source /opt/ros/<DISTRO>/setup.bash" >> ~/.bashrc
      source ~/.bashrc

In case you make use of a virtual environment, move to the directory containing the ``.venv`` and
add ``source /opt/ros/<DISTRO>/setup.bash`` to the activation script before activating the environment with
this line:

.. code:: shell

      echo "source /opt/ros/<DISTRO>/setup.bash" >> .venv/bin/activate

Cite EAGERx
===============
If you are using EAGERx for your scientific publications, please cite:

.. code:: bibtex

    @article{eagerx,
        author  = {van der Heijden, Bas and Luijkx, Jelle, and Ferranti, Laura and Kober, jens and Babuska, Robert},
        title = {EAGER: Engine Agnostic Gym Environment for Robotics},
        year = {2022},
        publisher = {GitHub},
        journal = {GitHub repository},
        howpublished = {\url{https://github.com/eager-dev/eagerx}}
    }

Maintainers
=================
EAGERx is currently maintained by Bas van der Heijden (`@bheijden <https://github.com/bheijden>`_) and Jelle Luijkx (`@jelledouwe <https://github.com/jelledouwe>`_).

How to contact us
=================
For any question, drop an e-mail at d.s.vanderheijden@tudelft.nl.

Acknowledgements
=================
EAGERx is funded by the `OpenDR <https://opendr.eu/>`_ Horizon 2020 project.
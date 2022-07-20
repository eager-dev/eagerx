.. image:: docs/_static/img/banner.png

**Streamlining the transfer of simulated robot learning to the real-world.**

.. image:: https://img.shields.io/badge/License-Apache_2.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
   :alt: license

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
   :target: https://github.com/psf/black
   :alt: codestyle

.. image:: https://readthedocs.org/projects/eagerx/badge/?version=master
   :target: https://eagerx.readthedocs.io/en/master/?badge=master
   :alt: Documentation Status

.. image:: https://github.com/eager-dev/eagerx/actions/workflows/ci.yml/badge.svg?branch=master
   :target: https://github.com/eager-dev/eagerx/actions/workflows/ci.yml
   :alt: Continuous Integration

.. image:: https://api.codeclimate.com/v1/badges/3146dce3dd4c3537834c/maintainability
   :target: https://codeclimate.com/github/eager-dev/eagerx/maintainability
   :alt: Maintainability

.. image:: https://api.codeclimate.com/v1/badges/3146dce3dd4c3537834c/test_coverage
   :target: https://codeclimate.com/github/eager-dev/eagerx/test_coverage
   :alt: Test Coverage

.. contents:: Table of Contents
    :depth: 2


What is EAGERx
==============
EAGERx (Engine Agnostic Graph Environments for Robotics) enables users to easily define new tasks, switch from one sensor to another,
and switch from simulation to reality with a single line of code by being invariant to the physics engine.
EAGERx explicitly addresses the differences in learning between simulation and reality,
with essential features for roboticists such as a safety layer, signal delay simulation, and controller switching for resets.
A single RL pipeline that works with both the simulated and real robots eliminates the chance for mismatches between the simulation and reality implementation.
The defined task follows the OpenAI Gym interface, so one can plug in algorithms from established RL libraries
(e.g., `Stable-baselines3 <https://github.com/DLR-RM/stable-baselines3>`_ ) to solve the task afterward, again minimizing implementation errors.

`Full documentation and tutorials available here <https://eagerx.readthedocs.io/en/master/>`_.

**We are currently working towards a first stable release!**

..
    TODO: ADD code example with gifs?
    Example
    =================

Installation
============

You can do a minimal installation of ``EAGERx`` with:

.. code:: shell

    pip3 install eagerx

We provide other options for installing EAGERx in `our documentation <https://eagerx.readthedocs.io/en/master/>`_ including
using Docker or Conda environment to have ROS1 setup.


Extras: GUI
-----------

To install the whole set of features, you will need additional packages.
You can install everything by running:

.. code:: shell

    pip3 install eagerx-gui

.. figure:: docs/_static/gif/gui.GIF
    :align: center
    :alt: alternate text
    :figclass: align-center

    The construction of an environment via the GUI.

Extras: training visualization
------------------------------

In robotics it is crucial to monitor the robot's behavior during the learning process.
Luckily, inter-node communication within EAGERx can always be listened to externally, so that any relevant information stream can be trivially monitored on-demand (e.g. with ``rqt_plot``).

.. note::
    ``rqt_plot`` is included in the ``desktop`` or ``desktop-full`` ROS installation. Follow the `ROS installation instructions <https://eagerx.readthedocs.io/en/latest/>`_ to install ROS.

..
  TODO: add example and gif of visualization.

Tutorials
=========
The following tutorials are currently available in the form of Google Colabs:

**Introduction to EAGERx**

- `Tutorial 1: Getting started <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/icra/getting_started.ipynb>`_
- `Tutorial 2: Advanced usage <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/icra/advanced_usage.ipynb>`_

The solutions are available `here <https://github.com/eager-dev/eagerx_tutorials/tree/master/tutorials/icra/solutions/>`_.

**Developer tutorials**

- `Tutorial 1: Environment Creation and Training with EAGERx <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/1_environment_creation.ipynb>`_
- `Tutorial 2: Reset and Step Function <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/2_reset_and_step.ipynb>`_
- `Tutorial 3: Space and Processors <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/3_space_and_processors.ipynb>`_
- `Tutorial 4: Nodes and Graph Validity <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/4_nodes.ipynb>`_
- `Tutorial 5: Adding Engine Support for an Object <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/5_engine_implementation.ipynb>`_
- `Tutorial 6: More Informative Rendering <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/6_rendering.ipynb>`_
- `Tutorial 7: Reset Routines <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/7_reset_routine.ipynb>`_
- Tutorial 8: Defining new objects (coming soon).
- Tutorial 9: Speeding-up training with multi-processing (coming soon).

For more information see the `docs <https://eagerx.readthedocs.io/en/master/guide/tutorials/colabs.html>`_ or the `eagerx_tutorials package <https://github.com/eager-dev/eagerx_tutorials>`_.

..
    Dependencies
    ============
    Below you find instructions for installing dependencies required for EAGERx.

    ROS
    ---

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
        sudo apt-get install ros-<DISTRO>-cv-bridge

    Make sure to source ``/opt/ros/<DISTRO>/setup.bash`` in the environment where you intend to ``eagerx`` in.
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
===========
If you are using EAGERx for your scientific publications, please cite:

.. code:: bibtex

    @article{eagerx,
        author  = {van der Heijden, Bas and Luijkx, Jelle, and Ferranti, Laura and Kober, Jens and Babuska, Robert},
        title = {EAGERx: Engine Agnostic Graph Environments for Robotics},
        year = {2022},
        publisher = {GitHub},
        journal = {GitHub repository},
        howpublished = {\url{https://github.com/eager-dev/eagerx}}
    }

Maintainers
===========
EAGERx is currently maintained by Bas van der Heijden (`@bheijden <https://github.com/bheijden>`_) and Jelle Luijkx (`@jelledouwe <https://github.com/jelledouwe>`_).

How to contact us
=================
Follow us on Twitter `@EagerxD <https://twitter.com/EagerxD>`_!

For any question, send an e-mail to eagerx.dev@gmail.com.

Acknowledgements
================
EAGERx is funded by the `OpenDR <https://opendr.eu/>`_ Horizon 2020 project.

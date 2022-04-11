.. _getting_started:
***************
Getting Started
***************

Installing EAGERx
=================

You can do a minimal installation of ``EAGERx`` with:

.. code:: shell

    pip3 install eagerx

.. note::
   EAGERx depends on a minimal ROS installation. Fortunately, you **can** use eagerx anywhere as you would any python package,
   so it does **not** impose a ROS package structure on your project.
   See `here <ROS_>`_ for installation instructions.

Extras: GUI
-----------

To install the whole set of features, you will need additional packages.
You can install everything by running:

.. code:: shell

    pip3 install eagerx-gui

..
  TODO: Add example and gif of GUI

Extras: training visualization
------------------------------
In robotics it is crucial to monitor the robot's behavior during the learning process.
Luckily, inter-node communication within EAGERx can always be listened to externally, so that any relevant information stream can be trivially monitored on-demand (e.g. with ``rqt_plot``).

.. note::
    ``rqt_plot`` is included in the ``desktop`` or ``desktop-full`` ROS installation. See `here <ROS_>`_ for installation instructions.

..
  TODO: add example and gif of visualization.

Runtime Dependencies
====================
Below you find instructions for installing runtime dependencies required for EAGERx.

ROS
---

See the `ROS Installation Options <http://wiki.ros.org/ROS/Installation>`_, or do the following.
By replacing ``<DISTRO>`` with the supported ROS distributions (``noetic``, ``melodic``),
and ``<PACKAGE>`` with the installation type (``ros-base``, ``desktop``, ``desktop-full``),
a minimal ros installation can be installed with:

.. warning:: Currently, eagerx only supports ROS1. ROS2 support will be added in future versions.

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

Known issues
============

- Using eagerx with anaconda can produce warnings (see below) when rendering or when using the GUI. This is a known issue that
  is caused by the interaction of pyqtgraph (used in the GUI) and opencv (used for rendering) with Qt libraries. Code seems not
  to break, so as a temporary fix, you are advised to suppress this error. Please file a bug report if eagerx/opencv/gui
  functionality actually breaks.

.. code:: shell

    QObject::moveToThread: Current thread (0x7fb6c4009eb0) is not the object's thread (0x7fb6c407cf40). Cannot move to
    target thread (0x7fb6c4009eb0).
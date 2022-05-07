.. _getting_started:
***************
Getting Started
***************

Installing EAGERx
=================

There are three installation options:

- Using *pip*
- From source
- Using docker

.. note::
   EAGERx depends on a minimal ROS1 installation.
   When installing EAGERx using pip or from source, ROS1 should be installed as well.
   Fortunately, you **can** use eagerx anywhere as you would any python package, so it does **not** impose a ROS package structure on your project.
   See `here <ROS1_>`_ for installation instructions.
   The docker image comes with an installation of ROS1.

Installation using *pip*
-----------------------

*Prerequisites*: Install `ROS1 <ROS1_>`_.

You can do a minimal installation of ``EAGERx`` with:

.. code:: shell

    pip3 install eagerx

Installation from source
------------------------

*Prerequisites*: Install `Poetry <Poetry_>`_ and `ROS1 <ROS1_>`_.

Clone the `eagerx repository <https://github.com/eager-dev/eagerx>`_ and go to its root:

.. code:: shell

  git clone git@github.com:eager-dev/eagerx.git
  cd eagerx

Install EAGERx:

.. code:: shell

  poetry install

Verify installation:

.. code:: shell

  poetry run python examples/example_openai.py

Installation using Docker
-------------------------

*Prerequisites*: `Install Docker <https://docs.docker.com/engine/install/>`_ and for GPU dockers `nvidia-docker <https://github.com/NVIDIA/nvidia-docker>`_.

In total, four docker images are available with EAGERx installed, i.e. two with a minimal installation of EAGERx and its dependencies (CPU and GPU) and two with `Stable Baselines 3 <https://stable-baselines3.readthedocs.io/en/master/index.html>`_ installed as well (CPU and GPU).
The dockers with Stable Baselines 3 also come with `tutorials on EAGERx <https://github.com/eager-dev/eagerx_tutorials>`_.

GPU Dockers
^^^^^^^^^^^

The GPU dockers require `nvidia-docker <https://github.com/NVIDIA/nvidia-docker>`_ and can be pulled as follows:

.. code:: shell

  sudo docker pull eagerx/eagerx

or with Stable Baselines 3 and the `tutorials <https://github.com/eager-dev/eagerx_tutorials>`_:

.. code:: shell

  sudo docker pull eagerx/eagerx-sb

The docker image can be run as follows:

.. code:: shell

  sudo docker run -it --rm --gpus all [image]

where [image] should be replaced with *eagerx/eagerx* or *eagerx/eagerx-sb*.

Verify that EAGERx is installed:

.. code:: shell
    python -c 'import eagerx'

CPU Dockers
^^^^^^^^^^^

The CPU only dockers can be pulled as follows:

.. code:: shell

  sudo docker pull [image]

where image should be replaced with *eagerx/eagerx-cpu* or *eagerx/eagerx-sb-cpu*.

Run the image with the command

.. code:: shell

  sudo docker run -it --rm [image]

where image should be replaced with *eagerx/eagerx-cpu* or *eagerx/eagerx-sb-cpu*.

Verify that EAGERx is installed:

.. code:: shell
    python -c 'import eagerx'

Extras: GUI
===========

To install the whole set of features, you will need additional packages.
There is for example a package available for visualizing the :class:`~eagerx.core.graph.Graph` and the :class:`~eagerx.core.graph_engine.EngineGraph`.
This `gui <https://github.com/eager-dev/eagerx_gui>`_ also allows to construct and modify a :class:`~eagerx.core.graph.Graph`.
You can install the gui by running:

.. code:: shell

    pip3 install eagerx-gui

.. note::

    The EAGERx docker images currently do not support gui functionality.

.. figure:: /_static/gif/gui.GIF
    :align: center
    :alt: alternate text
    :figclass: align-center

    The construction of an environment via the GUI.

Extras: training visualization
==============================

In robotics it is crucial to monitor the robot's behavior during the learning process.
Luckily, inter-node communication within EAGERx can always be listened to externally, so that any relevant information stream can be trivially monitored on-demand (e.g. with ``rqt_plot``).

.. note::
    ``rqt_plot`` is included in the ``desktop`` or ``desktop-full`` ROS1 installation.
    See `here <ROS1_>`_ for installation instructions.
    The docker images do not support visualization using ``rqt_plot``.

..
  TODO: add example and gif of visualization.

Other Dependencies
====================
Below you find instructions for installing dependencies (optionally) required by EAGERx.

Poetry
------
Poetry is a tool for dependency management and packaging in Python.
It allows you to declare the libraries your project depends on and it will manage (install/update) them for you.
We advise contributors to use this tool when developing an EAGERx package to leverage the pre-build CI workflow we have setup in the template package.
However, this is **not** a requirement and a simple `pip install` to install all eagerx package dependencies into your project's (virtual) Python environment will also work.

For installation on osx / linux / bashonwindows, simply run:

.. code:: shell

    curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -

For more installation instructions, see `here <https://python-poetry.org/docs/#installation>`_.

ROS1
----

See the `ROS1 Installation Options <http://wiki.ros.org/ROS/Installation>`_, or do the following.
By replacing ``<DISTRO>`` with the supported ROS1 distributions (``noetic``, ``melodic``),
and ``<PACKAGE>`` with the installation type (``ros-base``, ``desktop``, ``desktop-full``),
a minimal ROS1 installation can be installed with:

.. warning:: Currently, eagerx only supports ROS1. ROS2 support will be added in future versions.

.. code:: shell

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl = if you haven't already installed curl
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

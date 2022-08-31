.. _colabs:

******
Colabs
******

Introduction to EAGERx
######################

The best way to get introduced to EAGERx is to play around with the |colab| tutorials that are available.
They also contain exercises that address common challenges of robotic reinforcement learning and how to overcome them using EAGERx.

The following introductory tutorials are available:

- `Tutorial 1: Getting started <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/icra/getting_started.ipynb>`_
- `Tutorial 2: Advanced usage <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/icra/advanced_usage.ipynb>`_

`The solutions are available in here <https://github.com/eager-dev/eagerx_tutorials/tree/master/tutorials/icra/solutions/>`_.

.. figure:: /_static/gif/quadruped.gif
  :align: center
  :width: 480
  :alt: alternate text
  :figclass: align-center

  In the advanced usage tutorial you will learn a quadruped to walk in circles within four minutes of training.

1. Getting Started
------------------

This tutorial covers:

- constructing a :class:`~eagerx.core.graph.Graph` and an environment using :class:`~eagerx.core.env.BaseEnv`,
- switching between different :class:`~eagerx.core.entities.Engine`,
- performing domain randomization.

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/icra/getting_started.ipynb

2. Advanced Usage
-----------------

In this notebook, you will learn to use EAGERx to create a gym-compatible environment.
This tutorial covers:

- how to initialize a robot (Go 1 Quadruped Robot).
- how to add pre-processing nodes (i.e. low-level controllers).
- how to fine-tune low-level controllers to achieve the desired behavior.
- how to (de)select various sensors to investigate its effect on the learning performance.

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/icra/advanced_usage.ipynb

Developer Tutorials
###################

Next to the introduction tutorials, a set of developer tutorials is also available:

- `Tutorial 1: Environment Creation and Training with EAGERx <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/1_environment_creation.ipynb>`_
- `Tutorial 2: Reset and Step <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/2_reset_and_step.ipynb>`_
- `Tutorial 3: Space and Processors <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/3_space_and_processors.ipynb>`_
- `Tutorial 4: Nodes and Graph Validity <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/4_nodes.ipynb>`_
- `Tutorial 5: Adding Engine Support for an Object <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/5_engine_implementation.ipynb>`_
- `Tutorial 6: Defining a new Object <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/6_objects.ipynb>`_
- `Tutorial 7: More Informative Rendering <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/7_rendering.ipynb>`_
- `Tutorial 8: Reset Routines <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/8_reset_routine.ipynb>`_

`The solutions are available in here <https://github.com/eager-dev/eagerx_tutorials/tree/master/tutorials/pendulum/solutions/>`_.

.. figure:: /_static/gif/pendulum.GIF
  :align: center
  :width: 480
  :alt: alternate text
  :figclass: align-center

  The tutorials cover common challenges of robotic reinforcement learning and how to overcome them using EAGERx.
  The classic control problem of swinging up an underactuated pendulum is used as an example.

.. |colab| image:: /_static/img/colab.svg

1. Environment Creation and Training
------------------------------------

This tutorial covers:

- Creating a :class:`~eagerx.core.graph.Graph` with an :class:`~eagerx.core.entities.Object`.
- How to use this :class:`~eagerx.core.graph.Graph` and a :class:`~eagerx.core.entities.Engine` to create an :class:`~eagerx.core.env.BaseEnv`.
- How to train a policy with the :class:`~eagerx.core.env.BaseEnv`.

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/1_environment_creation.ipynb


2. Reset and Step
-----------------

This tutorial covers:

- Extracting observations in the :attr:`~eagerx.core.env.BaseEnv.step`
- Resetting states using :func:`~eagerx.core.env.BaseEnv.reset`
- The `window` argument of the :func:`~eagerx.core.graph.Graph.connect` method
- Simulating delays using the `delay` argument of the :func:`~eagerx.core.graph.Graph.connect` method

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/2_reset_and_step.ipynb

3. Space and Processors
-----------------------

This tutorial covers:

- How to specify a :class:`~eagerx.core.space.Space`
- Creating a custom :class:`~eagerx.core.entities.Processor`
- How to add a :class:`~eagerx.core.entities.Processor`


.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/3_space_and_processors.ipynb

4. Nodes and Graph Validity
---------------------------

This tutorial covers:

- Creating a :class:`~eagerx.core.entities.Node`
- Adding a :class:`~eagerx.core.entities.Node` to the :class:`~eagerx.core.graph.Graph`
- Checking the validity of the :class:`~eagerx.core.graph.Graph`
- How to make the :class:`~eagerx.core.graph.Graph` valid (DAG)

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/4_nodes.ipynb

5. Adding Engine Support for an Object
--------------------------------------

This tutorial covers:

- Adding an engine-specific implementation to an :class:`~eagerx.core.entities.Object`
- Initializing the corresponding :class:`~eagerx.core.entities.Engine`
- Train with the newly added engine-specific implementation

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/5_engine_implementation.ipynb

6. Defining a new Object
------------------------

This tutorial covers:

- Defining a new :class:`~eagerx.core.entities.Object`

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/6_objects.ipynb


7. More Informative Rendering
-----------------------------

- Create a layover :class:`~eagerx.core.entities.Node` that augments a raw image sensors
- Connect the layover :class:`~eagerx.core.entities.Node` and use it for rendering
- Demonstrate that rendering is agnostic to the selected physics-engine

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/6_rendering.ipynb


8. Reset Routines
-----------------

- Defining the reset routine with a :class:`~eagerx.core.entities.ResetNode`
- Reset the :class:`~eagerx.core.entities.Object`'s with the reset routine.

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/7_reset_routine.ipynb

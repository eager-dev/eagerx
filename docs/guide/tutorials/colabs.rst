.. _colabs:

******
Colabs
******

The best way to get introduced to EAGERx is to play around with the |colab| tutorials that are available.
They also contain exercises that address common challenges of robotic reinforcement learning and how to overcome them using EAGERx.

The following tutorials are currently available:

- `Tutorial 1: Environment Creation and Training with EAGERx <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/1_environment_creation.ipynb>`_
- `Tutorial 2: Reset and Step Function <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/2_reset_and_step.ipynb>`_
- `Tutorial 3: Converters <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/3_converters.ipynb>`_
- `Tutorial 4: Nodes and Graph Validity <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/4_nodes.ipynb>`_
- `Tutorial 5: Adding Engine Support for an Object <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/5_engine_implementation.ipynb>`_
- `Tutorial 6: More Informative Rendering <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/6_rendering.ipynb>`_
- `Tutorial 7: Reset Routines <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/7_reset_routine.ipynb>`_

.. figure:: /_static/gif/pendulum.GIF
  :align: center
  :width: 480
  :alt: alternate text
  :figclass: align-center

  The tutorials cover common challenges of robotic reinforcement learning and how to overcome them using EAGERx.
  The classic control problem of swinging up an underactuated pendulum is used as an example.

.. |colab| image:: /_static/img/colab.svg

1. Environment Creation and Training
####################################

This tutorial covers:

- Creating a :class:`~eagerx.core.graph.Graph` with an :class:`~eagerx.core.entities.Object`.
- How to use this :class:`~eagerx.core.graph.Graph` and a :class:`~eagerx.core.entities.Engine` to create an :class:`~eagerx.core.env.EagerxEnv`.
- How to train a policy with the :class:`~eagerx.core.env.EagerxEnv`.

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/1_environment_creation.ipynb


2. Reset and Step Function
##########################

This tutorial covers:

- Extracting observations in the :attr:`~eagerx.core.env.EagerxEnv.step_fn`
- Resetting states using the :attr:`~eagerx.core.env.EagerxEnv.reset_fn`
- The `window` argument of the :func:`~eagerx.core.graph.Graph.connect` method
- Simulating delays using the `delay` argument of the :func:`~eagerx.core.graph.Graph.connect` method

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/2_reset_and_step.ipynb

3. Converters
#############

This tutorial covers:

- The three different converters, i.e. :class:`~eagerx.core.entities.SpaceConverter`, :class:`~eagerx.core.entities.Processor` and :class:`~eagerx.core.entities.Converter`
- Specifying the parameters of converters
- Creating a custom :class:`~eagerx.core.entities.SpaceConverter`

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/3_converters.ipynb

4. Nodes and Graph Validity
###########################

This tutorial covers:

- Creating a :class:`~eagerx.core.entities.Node`
- Adding a :class:`~eagerx.core.entities.Node` to the :class:`~eagerx.core.graph.Graph`
- Checking the validity of the :class:`~eagerx.core.graph.Graph`
- How to make the :class:`~eagerx.core.graph.Graph` valid (DAG)

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/4_nodes.ipynb

5. Adding Engine Support for an Object
######################################

This tutorial covers:

- Adding an engine-specific implementation to an :class:`~eagerx.core.entities.Object`
- Initializing the corresponding :class:`~eagerx.core.entities.Engine`
- Train with the newly added engine-specific implementation

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/5_engine_implementation.ipynb

6. More Informative Rendering
#############################

- Create a layover :class:`~eagerx.core.entities.Node` that augments a raw image sensors
- Connect the layover :class:`~eagerx.core.entities.Node` and use it for rendering
- Demonstrate that rendering is agnostic to the selected physics-engine

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/6_rendering.ipynb


7. Reset Routines
#################

- Defining the reset routine with a :class:`~eagerx.core.entities.ResetNode`
- Reset the :class:`~eagerx.core.entities.Object`'s with the reset routine.

.. image:: /_static/img/colab-badge.svg
  :target: https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/7_reset_routine.ipynb

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
- How to use this :class:`~eagerx.core.graph.Graph` and a :class:`~eagerx.core.entities.Bridge` to create an :class:`~eagerx.core.env.EagerxEnv`.
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

.. _pendulum:

******************
Creating an Object
******************

In this section, we will discuss the concept of :class:`~eagerx.core.entities.Object` within EAGERx by going through the steps of creating the :class:`~eagerx_dcsc_setups.pendulum.objects.Pendulum` object.
For this *Pendulum* :class:`~eagerx.core.entities.Object` we will create two bridge implementations, i.e. for the `OdeBridge <https://github.com/eager-dev/eagerx_ode>`_ and for the `RealBridge <https://github.com/eager-dev/eagerx_reality>`_.
This will allow us to use the same :class:`~eagerx.core.entities.Object` for both simulated and real experiments.
We will start by implementing the agnostic part of the *Pendulum* (stuff that is independent from the :class:`~eagerx.core.entities.Bridge` that is used).
Next, we will implement everything related to the *OdeBridge* and finally we create the implementation for the *RealBridge*.

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_

.. figure:: /_static/img/object.svg
  :align: center
  :alt: alternate text
  :figclass: align-center

  In this section we will discuss the concept of a :class:`~eagerx.core.entities.Object`.
  An :class:`~eagerx.core.entities.Object` consists of a collection of :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states`.
  Within the :class:`~eagerx.core.entities.Object`, we also define the :class:`~eagerx.core.graph_engine.EngineGraph` by creating a graph of nodes of type :class:`~eagerx.core.entities.EngineNode` for each physics engine.

.. toctree::
   :maxdepth: 2
   :caption: Table of Contents

   ./pendulum_agnostic
   ./pendulum_sim
   ./engine_graph
   ./pendulum_real

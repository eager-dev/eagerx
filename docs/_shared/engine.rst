******
Engine
******

In this section we will describe how to create an engine.
We will show this by going through the steps of creating the OdeEngine, which allows to simulate systems based on known ordinary differential equations (ODEs).
First, we have created an empty package using the template as described here in the contributing to EAGERx section under package creation.

We will create three Python files that together will define the OdeEngine, i.e.:

* `engine.py <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine.py>`_: here we will define the engine that performs integration of the ODEs.
* `engine_nodes.py <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_nodes.py>`_: here we will define the engine nodes of the OdeEngine.
* `engine_states.py <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_states.py>`_: here we will define the engine states of the OdeEngine.

The creation of these files will be discussed in the following sections.

.. toctree::
   :maxdepth: 1

   ./engine_node
   ./engine_nodes
   ./engine_state

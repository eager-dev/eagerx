******
Bridge
******

In this section we will describe how to create a bridge.
We will show this by going through the steps of creating the OdeBridge, which allows to simulate systems based on known ordinary differential equations (ODEs).
First, we have created an empty package using the template as described here in the contributing to EAGERx section under package creation.

We will create three Python files that together will define the OdeBridge, i.e.:

* *bridge.py*: here we will define the bridge that performs integration of the ODEs.
* *engine_nodes.py*: here we will define the engine nodes of the OdeBridge.
* *engine_states.py*: here we will define the engine states of the OdeBridge.

.. toctree::
   :maxdepth: 1

   ./bridge_node
   ./engine_node
   ./engine_state

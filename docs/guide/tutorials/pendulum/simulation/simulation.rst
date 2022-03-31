***********************
Simulating the Pendulum
***********************

We want to be able to simulate the dynamics of the pendulum, such that we can train in simulation.
In this way, we will be able to obtain a policy in simulation that we can later fine tune on the real system.
The dynamics of a pendulum are well understood.
Therefore, we can express the dynamics of the pendulum as ordinary differential equations (ODEs) and use existing ODE solvers to simulate dynamics.
In order to have a generic implementation that can be used for other systems as well, we will create a bridge that simulates a system based on a given ODE.
We will call this bridge the OdeBridge. In the following section we will show how this bridge is created.


Creating the OdeBridge
======================

In this section we will describe how to create an EAGERx package, in this case the eagerx_ode package.
Since the OdeBridge will be a generic bridge that can be useful for others, we will create a public repository for the OdeBridge.
We will start by creating a new repository for this Python package, using `the template that is available here <https://github.com/eager-dev/eagerx_template>`_.

The following section describes how this is done:

.. toctree::
   :maxdepth: 1

   ./package_creation
   ./bridge

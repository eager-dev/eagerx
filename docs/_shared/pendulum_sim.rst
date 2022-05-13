.. _engine_specific_ode_engine:
***************************
Engine-Specific (OdeEngine)
***************************

Having defined the agnostic parameters of the *Pendulum*, we can now specify the engine-specific implementations.
In this case, we will create an implementation for the `*OdeEngine* <https://github.com/eager-dev/eagerx_ode>`_.

ode_engine
##########

Engine-specific implementations can be created by adding a method to an :class:`~eagerx.core.entities.Object`, e.g. :func:`~eagerx.core.entities.Object.example_engine`.
Here we will define which :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState` will be used for which :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states`.
In our case, we will use the *OdeParameters* and *OdeEngineState* (`which are defined here <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_states.py>`_) for the *model_parameters* and *model_state* :attr:`~eagerx.core.specs.ObjectSpec.states`, respectively.
We will use the *OdeOutput*, *ActionApplied* and *OdeInput* (`which are defined here <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_nodes.py>`_) for the *pendulum_output* sensor, the *action_applied* and *pendulum_input* actuators, respectively.
Also, the *image* sensor will render the pendulum.
For this, we will make use of the *PendulumImage* :class:`~eagerx.core.entities.engine_node` (`which is defined here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/ode/engine_nodes.py>`_).
For creating these states and nodes, we use the :func:`~eagerx.core.entities.EngineState.make` and :func:`~eagerx.core.entities.EngineNode.make` methods.
Furthermore, we specify where the ODE of the pendulum can be found (`which is defined here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/ode/pendulum_ode.py>`_).
Finally, after creating these nodes, we construct an :class:`~eagerx.core.graph_engine.EngineGraph` using these nodes by connecting them to each other.

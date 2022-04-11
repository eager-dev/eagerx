.. _bridge_specific_ode_bridge:
***************************
Bridge-Specific (OdeBridge)
***************************

Having defined the agnostic parameters of the *Pendulum*, we can now specify the bridge-specific implementations.
In this case, we will create an implementation for the `*OdeBridge* <https://github.com/eager-dev/eagerx_ode>`_.

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_

ode_bridge
##########

Bridge-specific implementations can be created by adding a method to an :class:`~eagerx.core.entities.Object`, e.g. :func:`~eagerx.core.entities.Object.example_bridge`.
Here we will define which :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState` will be used for which :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states`.
In our case, we will use the *OdeParameters* and *OdeEngineState* (`which are defined here <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_states.py>`_) for the *model_parameters* and *model_state* :attr:`~eagerx.core.specs.ObjectSpec.states`, respectively.
We will use the *OdeOutput*, *ActionApplied* and *OdeInput* (`which are defined here <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_nodes.py>`_) for the *pendulum_output* sensor, the *action_applied* and *pendulum_input* actuators, respectively.
Also, the *image* sensor will render the pendulum.
For this, we will make use of the *PendulumImage* :class:`~eagerx.core.entities.engine_node` (`which is defined here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/ode/engine_nodes.py>`_).
For creating these states and nodes, we use the :func:`~eagerx.core.entities.EngineState.make` and :func:`~eagerx.core.entities.EngineNode.make` methods.
Furthermore, we specify where the ODE of the pendulum can be found (`which is defined here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/ode/pendulum_ode.py>`_).
Finally, after creating these nodes, we construct an :class:`~eagerx.core.graph_engine.EngineGraph` using these nodes by connecting them to each other.

::

  @staticmethod
  @register.bridge(entity_id, OdeBridge)  # This decorator pre-initializes bridge implementation with default object_params
  def ode_bridge(spec: ObjectSpec, graph: EngineGraph):
      """Engine-specific implementation (OdeBridge) of the object."""
      # Import any object specific entities for this bridge
      import eagerx_dcsc_setups.pendulum.ode  # noqa # pylint: disable=unused-import

      # Set object arguments (nothing to set here in this case)
      spec.OdeBridge.ode = "eagerx_dcsc_setups.pendulum.ode.pendulum_ode/pendulum_ode"
      # Set default params of pendulum ode [J, m, l, b0, K, R, c, a].
      spec.OdeBridge.ode_params = [0.000189238, 0.0563641, 0.0437891, 0.000142205, 0.0502769, 9.83536, 1.49553, 0.00183742]

      # Create engine_states (no agnostic states defined in this case)
      spec.OdeBridge.states.model_state = EngineState.make("OdeEngineState")
      spec.OdeBridge.states.model_parameters = EngineState.make("OdeParameters", list(range(7)))

      # Create sensor engine nodes
      obs = EngineNode.make("OdeOutput", "pendulum_output", rate=spec.sensors.pendulum_output.rate, process=2)
      image = EngineNode.make(
          "PendulumImage", "image", shape=spec.config.render_shape, rate=spec.sensors.image.rate, process=0
      )

      # Create actuator engine nodes
      action = EngineNode.make(
          "OdeInput", "pendulum_actuator", rate=spec.actuators.pendulum_input.rate, process=2, default_action=[0]
      )

      # Connect all engine nodes
      graph.add([obs, image, action])
      graph.connect(source=obs.outputs.observation, sensor="pendulum_output")
      graph.connect(source=obs.outputs.observation, target=image.inputs.theta)
      graph.connect(source=image.outputs.image, sensor="image")
      graph.connect(actuator="pendulum_input", target=action.inputs.action)

      # Add action applied
      applied = EngineNode.make("ActionApplied", "applied", rate=spec.sensors.action_applied.rate, process=0)
      graph.add(applied)
      graph.connect(source=action.outputs.action_applied, target=applied.inputs.action_applied, skip=True)
      graph.connect(source=applied.outputs.action_applied, sensor="action_applied")

.. note::
  Mind the usage of the :func:`~eagerx.core.register.bridge` decorator.
  Also, we want to point out that the API for creating the :class:`~eagerx.core.graph_engine.EngineGraph` is similar to the one from :class:`~eagerx.core.graph.Graph`.

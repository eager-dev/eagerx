.. _bridge_specific_real_bridge:
****************************
Bridge-Specific (RealBridge)
****************************

Having defined the agnostic parameters of the *Pendulum*, we can now specify the bridge-specific implementations.
In this case, we will create an implementation for the `RealBridge <https://github.com/eager-dev/eagerx_reality>`_.

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_

real_bridge
##########

Bridge-specific can be created by adding a method to an :mod:`~eagerx.core.entities.Object`, e.g. :func:`~eagerx.core.entities.Object.example_bridge`.
In this case, we create an implementation for the *RealBridge*, to be able to perform experiments with the real system.
For this, we use the `engine nodes that we have created here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/real/engine_nodes.py>`_.
Also, we will be using the `engine states that we have created here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/real/engine_states.py>`_.
Finally, we will construct an :mod:`~eagerx.core.graph_engine.EngineGraph` with the created nodes.

::

  @staticmethod
  @register.bridge(entity_id, RealBridge)  # This decorator pre-initializes bridge implementation with default object_params
  def real_bridge(spec: ObjectSpec, graph: EngineGraph):
      """Engine-specific implementation (RealBridge) of the object."""
      # Import any object specific entities for this bridge
      import eagerx_dcsc_setups.pendulum.real  # noqa # pylint: disable=unused-import

      # Couple engine states
      spec.RealBridge.states.model_state = EngineState.make("RandomActionAndSleep", sleep_time=1.0, repeat=1)

      # Create sensor engine nodes
      obs = EngineNode.make("PendulumOutput", "pendulum_output", rate=spec.sensors.pendulum_output.rate, process=0)
      applied = EngineNode.make("ActionApplied", "applied", rate=spec.sensors.action_applied.rate, process=0)
      image = EngineNode.make(
          "CameraRender",
          "image",
          camera_idx=spec.config.camera_index,
          shape=spec.config.render_shape,
          rate=spec.sensors.image.rate,
          process=0,
      )

      # Create actuator engine nodes
      action = EngineNode.make("PendulumInput", "pendulum_input", rate=spec.actuators.pendulum_input.rate, process=0)

      # Connect all engine nodes
      graph.add([obs, applied, image, action])
      graph.connect(source=obs.outputs.pendulum_output, sensor="pendulum_output")
      graph.connect(source=action.outputs.action_applied, target=applied.inputs.action_applied, skip=True)
      graph.connect(source=applied.outputs.action_applied, sensor="action_applied")
      graph.connect(source=image.outputs.image, sensor="image")
      graph.connect(actuator="pendulum_input", target=action.inputs.pendulum_input)

.. note::
  Mind the use of the :func:`~eagerx.core.register.bridge` decorator, which creates the link to the corresponding bridge.
  Therefore, the name of the *real_bridge* method is irrelevant, i.e. the link to the *RealBridge* is defined by the aforementioned decorator.
  Also note that we are importing :mod:`eagerx_dcsc_setups.pendulum.real`.
  During the import, the engine nodes of this module are registered and therefore we can use the :func:`~eagerx.core.entities.EngineNode.make` and :func:`~eagerx.core.entities.EngineState.make` methods with the IDs to create these nodes (e.g. PendulumOutput).

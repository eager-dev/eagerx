************
Engine Graph
************

In this section we will discuss the concept of the :class:`~eagerx.core.graph_engine.EngineGraph`.
We will do this by going through an example.
In this case, we will construct the :class:`~eagerx.core.graph_engine.EngineGraph` for the :ref:`OdeEngine <ode_engine>`. implementation within the :ref:`Pendulum <pendulum>` :class:`~eagerx.core.entities.Object`.

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_

.. figure:: /_static/img/engine_graph.svg
  :align: center
  :alt: alternate text
  :figclass: align-center

  The :class:`~eagerx.core.graph_engine.EngineGraph` defines how the nodes of type :class:`~eagerx.core.entities.EngineNode` are connected to eachother within an engine-specific implementation of an :class:`~eagerx.core.entities.Object`.

Constructing the :class:`~eagerx.core.graph_engine.EngineGraph`
###############################################################

The :class:`~eagerx.core.graph_engine.EngineGraph` defines how the nodes of type :class:`~eagerx.core.entities.EngineNode` are connected to eachother within an engine-specific implementation of an :class:`~eagerx.core.entities.Object`.
Therefore, it should be constructed within an engine-specific implementation of an :class:`~eagerx.core.entities.Object.

In this case, we will construct an :class:`~eagerx.core.graph_engine.EngineGraph` with three sensors, i.e. *pendulum_output*, *image* and *action_applied* and one actuator, i.e. *pendulum_input*.


::

  @staticmethod
  @register.engine(entity_id, OdeEngine)  # This decorator pre-initializes engine implementation with default object_params
  def ode_engine(spec: ObjectSpec, graph: EngineGraph):
      """Engine-specific implementation (OdeEngine) of the object."""
      # Import any object specific entities for this engine
      import eagerx_dcsc_setups.pendulum.ode  # noqa # pylint: disable=unused-import

      # Set object arguments (nothing to set here in this case)
      spec.OdeEngine.ode = "eagerx_dcsc_setups.pendulum.ode.pendulum_ode/pendulum_ode"
      # Set default params of pendulum ode [J, m, l, b0, K, R, c, a].
      spec.OdeEngine.ode_params = [0.000189238, 0.0563641, 0.0437891, 0.000142205, 0.0502769, 9.83536, 1.49553, 0.00183742]

      # Create engine_states (no agnostic states defined in this case)
      spec.OdeEngine.states.model_state = EngineState.make("OdeEngineState")
      spec.OdeEngine.states.model_parameters = EngineState.make("OdeParameters", list(range(7)))

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
  Mind the usage of the :func:`~eagerx.core.register.engine` decorator.
  Also, we want to point out that the API for creating the :class:`~eagerx.core.graph_engine.EngineGraph` is similar to the one from :class:`~eagerx.core.graph.Graph`.

Visualization and Validation
############################

We can use the `GUI <https://github.com/eager-dev/eagerx_gui>`_ to inspect the :class:`~eagerx.core.graph_engine.EngineGraph`.
This can be done by calling the :func:`~eagerx.core.graph_engine.EngineGraph.gui` method:

::

  graph.gui()

Also, after using the :func:`~eagerx.core.entities.Object.make` method to make an object, we can visualize the :class:`~eagerx.core.graph_engine.EngineGraph`, using the :func:`~eagerx.core.specs.EngineSpec.gui` method:

::

  import eagerx
  import eagerx_dcsc_setups

  pendulum = eagerx.Object.make("Pendulum", "pendulum")
  pendulum.gui(engine_id="OdeEngine")

.. note::
  We have to call the :func:`~eagerx.core.specs.EngineSpec.gui` method with the argument `engine_id`, since an :class:`~eagerx.core.entities.Object` can have implementations for more than one :class:`~eagerx.core.entities.Engine`, where each has its own :class:`~eagerx.core.graph_engine.EngineGraph`.

When clicking *Show Graph*, the output should look similar to the image below:

.. figure:: /_static/img/pendulum_engine_graph.svg
  :align: center
  :alt: alternate text
  :figclass: align-center

  The :class:`~eagerx.core.graph_engine.EngingeGraph` for the *OdeEngine* of the *Pendulum* :class:`~eagerx.core.entities.Object`.
  Here we can see three sensors (*pendulum_output*, *action_applied*, *image*) and one actuator (*pendulum_input*).
  Note that each :class:`~eagerx.core.entities.EngineNode` with the input *tick* is synchronized with the :class:`~eagerx.core.entities.Engine`.

We can also check whether the :class:`~eagerx.core.graph_engine.EngineGraph` is valid by clicking *Check Validity*.
Among other things, this checks whether the graph is a directed acyclical graph (DAG).
We can perform the same check using the :func:`~eagerx.core.graph_engine.EngineGraph.is_valid` method.

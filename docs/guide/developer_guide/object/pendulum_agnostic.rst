********
Agnostic
********

Each :mod:`~eagerx.core.entities.Object` requires an agnostic implementation.
With agnostic, we mean agnostic to the type of bridge that is used.
This concerns for example the action and observation spaces of the :mod:`~eagerx.core.entities.Object`, which are the same no matter whether the system is simulated or not.

An :mod:`~eagerx.core.entities.Object` has two abstract classes:

- :func:`~eagerx.core.entities.Object.agnostic`
- :func:`~eagerx.core.entities.Object.spec`

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_

agnostic
========

The :func:`~eagerx.core.entities.Object.agnostic` method should be used for defining the information that is agnostic of the bridge that is being used.
Here we specify what :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states` the :mod:`~eagerx.core.entities.Object` has.
An actuator can be used to apply an action in the environment, a sensor can obtain observations, while a state is something that can be reset before starting an episode.
In our case, we have three sensors (*pendulum_output*, *action_applied* and *image*), one actuator (*pendulum_input*) and two states (*model_state*, *model_parameters*).
We use the *model_state* to reset the angle and angular velocity of the pendulum during a reset, while we use the *model_parameters* state to randomize the model parameters over the episodes in order to improve robustness against model inaccuracies.
For each of these :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states`, we will specify rates, windows and space converters.
The rates define at which rate the callback of that entity is called, window sizes determines the window size for incoming messages, while space converters define how to convert the messages to an Openai Gym space.
More information on these parameters is available at the API Reference sections on :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states`.

::

  # ROS IMPORTS
  from std_msgs.msg import Float32MultiArray
  from sensor_msgs.msg import Image

  # EAGERx IMPORTS
  from eagerx_reality.bridge import RealBridge
  from eagerx_ode.bridge import OdeBridge
  from eagerx import Object, EngineNode, SpaceConverter, EngineState, Processor
  from eagerx.core.specs import ObjectSpec
  from eagerx.core.graph_engine import EngineGraph
  import eagerx.core.register as register


  class Pendulum(Object):
    entity_id = "Pendulum"

    @staticmethod
    @register.sensors(pendulum_output=Float32MultiArray, action_applied=Float32MultiArray, image=Image)
    @register.actuators(pendulum_input=Float32MultiArray)
    @register.engine_states(model_state=Float32MultiArray, model_parameters=Float32MultiArray)
    @register.config(always_render=False, render_shape=[480, 480], camera_index=0)
    def agnostic(spec: ObjectSpec, rate):
        """Agnostic definition of the Pendulum"""
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Set observation properties: (space_converters, rate, etc...)
        spec.sensors.pendulum_output.rate = rate
        spec.sensors.pendulum_output.space_converter = SpaceConverter.make(
            "Space_AngleDecomposition", low=[-1, -1, -9], high=[1, 1, 9], dtype="float32"
        )

        spec.sensors.action_applied.rate = rate
        spec.sensors.action_applied.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=[-3], high=[3], dtype="float32"
        )

        spec.sensors.image.rate = 15
        spec.sensors.image.space_converter = SpaceConverter.make(
            "Space_Image", low=0, high=1, shape=spec.config.render_shape, dtype="float32"
        )

        # Set actuator properties: (space_converters, rate, etc...)
        spec.actuators.pendulum_input.rate = rate
        spec.actuators.pendulum_input.window = 1
        spec.actuators.pendulum_input.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=[-3], high=[3], dtype="float32"
        )

        # Set model_state properties: (space_converters)
        spec.states.model_state.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=[-3.14159265359, -9], high=[3.14159265359, 9], dtype="float32"
        )

        # Set model_parameters properties: (space_converters) # [J, m, l, b0, K, R, c, a]
        fixed = [0.000189238, 0.0563641, 0.0437891, 0.000142205, 0.0502769, 9.83536, 1.49553, 0.00183742]
        diff = [0, 0, 0, 0.08, 0.08, 0.08, 0.08]  # Percentual delta with respect to fixed value
        low = [val - diff * val for val, diff in zip(fixed, diff)]
        high = [val + diff * val for val, diff in zip(fixed, diff)]
        # low = [1.7955e-04, 5.3580e-02, 4.1610e-02, 1.3490e-04, 4.7690e-02, 9.3385e+00, 1.4250e+00, 1.7480e-03]
        # high = [1.98450e-04, 5.92200e-02, 4.59900e-02, 1.49100e-04, 5.27100e-02, 1.03215e+01, 1.57500e+00, 1.93200e-03]
        spec.states.model_parameters.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=low, high=high, dtype="float32"
        )

.. note::
  Mind the use of the :func:`~eagerx.core.register.sensors`, :func:`~eagerx.core.register.actuators`, :func:`~eagerx.core.register.engine_states` and :func:`~eagerx.core.register.config` decorators.
  Registration is required to be able to set the :mod:`~eagerx.core.specs.ObjectSpec`.
  Also, note that we import :mod:`eagerx.converters`.
  While it might look like this import is unused, it actually registers the converters from that module, such that we can use them.
  The :mod:`~eagerx.converters.space_ros_converters.Space_Float32MultiArray` and :mod:`~eagerx.converters.space_ros_converters.Space_Image` can therefore be used.
  The :mod:`Space_AngleDecomposition` space converter can be used because it is imported during initialization of the package in which the object is defined.
  `This space converter is defined here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/converters.py>`_


spec
========

The :func:`~eagerx.core.specs.ObjectSpec` specifies how :mod:`~eagerx.core.env.EagerxEnv` should initialize the object.
Here we can for example specify what :attr:`~eagerx.core.specs.ObjectSpec.actuators`, :attr:`~eagerx.core.specs.ObjectSpec.sensors` and :attr:`~eagerx.core.specs.ObjectSpec.states` should be used by default, because this does not necessarily have to be all of them.
Per default, we will e.g. use the *model_state* :mod:`~eagerx.core.entities.EngineState` only.

::

  @staticmethod
  @register.spec(entity_id, Object)
  def spec(
      spec: ObjectSpec, name: str, sensors=None, states=None, rate=30, always_render=False, render_shape=None, camera_index=2
  ):
      """Object spec of Pendulum"""
      # Performs all the steps to fill-in the params with registered info about all functions.
      Pendulum.initialize_spec(spec)

      # Modify default agnostic params
      # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
      spec.config.name = name
      spec.config.sensors = sensors if sensors else ["pendulum_output", "action_applied", "image"]
      spec.config.actuators = ["pendulum_input"]
      spec.config.states = states if states else ["model_state"]

      # Add registered agnostic params
      spec.config.always_render = always_render
      spec.config.render_shape = render_shape if render_shape else [480, 480]
      spec.config.camera_index = camera_index

      # Add bridge implementation
      Pendulum.agnostic(spec, rate)


.. note::
  Mind the usage of the :func:`~eagerx.core.register.spec` for initialization of the :mod:`~eagerx.core.specs.ObjectSpec`.
  Also, the parameters that were added to the :func:`~eagerx.core.register.config` (*always_render*, *render_shape*, *camera_index*), become arguments to the :func:`~eagerx.core.entities.Object.spec` method.

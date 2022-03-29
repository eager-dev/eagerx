*******************
Pendulum (agnostic)
*******************

Each :mod:`~eagerx.core.entities.Object` requires an agnostic implementation.
With agnostic, we mean agnostic to the type of bridge that is used.
This concerns for example the action and observation spaces of the :mod:`~eagerx.core.entities.Object`, which are the same no matter whether the system is simulated or not.

An :mod:`~eagerx.core.entities.Object` has two abstract classes:

- :func:`~eagerx.core.entities.Object.agnostic`
- :func:`~eagerx.core.entities.Object.spec`

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_

spec
====

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


agnostic
========

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

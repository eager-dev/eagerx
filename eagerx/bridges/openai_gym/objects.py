# OTHER IMPORTS
from typing import Optional, List

# ROS IMPORTS
from std_msgs.msg import Float32MultiArray, Bool, Float32
from sensor_msgs.msg import Image

# EAGERx IMPORTS
from eagerx.bridges.openai_gym.bridge import GymBridge
from eagerx.core.entities import Object, EngineNode, SpaceConverter
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class GymObject(Object):
    entity_id = "GymObject"

    @staticmethod
    @register.sensors(observation=Float32MultiArray, reward=Float32, done=Bool, image=Image)
    @register.actuators(action=Float32MultiArray)
    @register.engine_states()
    @register.config(env_id=None, always_render=False, default_action=None, render_shape=[200, 200])
    def agnostic(spec: ObjectSpec, rate):
        """Agnostic definition of the GymObject"""
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Set observation space_converters
        spec.sensors.observation.space_converter = SpaceConverter.make(
            "GymSpace_Float32MultiArray", gym_id=spec.config.env_id, space="observation"
        )
        spec.sensors.reward.space_converter = SpaceConverter.make("Space_Float32", low=-99999, high=9999, dtype="float32")
        spec.sensors.done.space_converter = SpaceConverter.make("Space_Bool")
        spec.sensors.image.space_converter = SpaceConverter.make(
            "Space_Image", low=0, high=1, shape=spec.config.render_shape, dtype="float32"
        )
        spec.actuators.action.space_converter = SpaceConverter.make(
            "GymSpace_Float32MultiArray", gym_id=spec.config.env_id, space="action"
        )

        # Set observation rates
        spec.sensors.observation.rate = rate
        spec.sensors.reward.rate = rate
        spec.sensors.done.rate = rate
        spec.sensors.image.rate = rate
        spec.actuators.action.rate = rate

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
        name: str,
        sensors: Optional[List[str]] = None,
        env_id: str = "Pendulum-v1",
        rate: float = 20.0,
        always_render: bool = False,
        default_action=None,
        render_shape: Optional[List[int]] = None,
    ):
        """Object spec of GymObject"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        GymObject.initialize_spec(spec)

        # Set default node params
        spec.config.name = name
        spec.config.sensors = sensors if isinstance(sensors, list) else ["observation", "reward", "done"]
        spec.config.actuators = ["action"]

        # Set custom node params
        spec.config.render_shape = render_shape if render_shape else [200, 200]
        spec.config.env_id = env_id
        spec.config.always_render = always_render

        # Add agnostic definition
        GymObject.agnostic(spec, rate)

    @staticmethod
    @register.bridge(entity_id, GymBridge)  # This decorator pre-initializes bridge implementation with default object_params
    def openai_gym(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (GymBridge) of the object."""
        # Set bridge arguments (nothing to set here in this case)
        spec.GymBridge.env_id = spec.config.env_id

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        obs = EngineNode.make("ObservationSensor", "obs", rate=spec.sensors.observation.rate, process=2)
        rwd = EngineNode.make("RewardSensor", "rwd", rate=spec.sensors.reward.rate, process=2)
        done = EngineNode.make("DoneSensor", "done", rate=spec.sensors.done.rate, process=2)
        image = EngineNode.make(
            "GymImage",
            "image",
            shape=spec.config.render_shape,
            always_render=spec.config.always_render,
            rate=spec.sensors.image.rate,
            process=2,
        )

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        action = EngineNode.make(
            "ActionActuator", "action", rate=spec.actuators.action.rate, process=2, zero_action=spec.config.default_action
        )

        # Connect all engine nodes
        graph.add([obs, rwd, done, image, action])
        graph.connect(source=obs.outputs.observation, sensor="observation")
        graph.connect(source=rwd.outputs.reward, sensor="reward")
        graph.connect(source=done.outputs.done, sensor="done")
        graph.connect(source=image.outputs.image, sensor="image")
        graph.connect(actuator="action", target=action.inputs.action)

        # Check graph validity (comment out)
        # graph.is_valid(plot=True)

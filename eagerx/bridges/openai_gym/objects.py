# OTHER IMPORTS
from typing import Optional, List

# ROS IMPORTS
from std_msgs.msg import Float32MultiArray, Bool, Float32
from sensor_msgs.msg import Image

# EAGERx IMPORTS
from eagerx.bridges.openai_gym.bridge import GymBridge
from eagerx.core.entities import Object, EngineNode, SpaceConverter
from eagerx.core.specs import ObjectSpec, AgnosticSpec, SpecificSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class GymObject(Object):
    @staticmethod
    @register.sensors(observation=Float32MultiArray, reward=Float32, done=Bool, image=Image)
    @register.actuators(action=Float32MultiArray)
    @register.simstates()
    @register.agnostic_params(
        gym_always_render=False,
        default_action=None,
        gym_rate=20,
        gym_env_id="Pendulum-v1",
        render_shape=[200, 200],
    )
    def agnostic(spec: AgnosticSpec):
        """Agnostic definition of the GymObject"""
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Set observation space_converters
        spec.sensors.observation.space_converter = SpaceConverter.make(
            "GymSpace_Float32MultiArray",
            gym_id="$(config gym_env_id)",
            space="observation",
        )
        spec.sensors.reward.space_converter = SpaceConverter.make("Space_Float32", low=-99999, high=9999, dtype="float32")
        spec.sensors.done.space_converter = SpaceConverter.make("Space_Bool")
        spec.sensors.image.space_converter = SpaceConverter.make(
            "Space_Image",
            low=0,
            high=1,
            shape="(default render_shape)",
            dtype="float32",
        )
        spec.actuators.action.space_converter = SpaceConverter.make(
            "GymSpace_Float32MultiArray", gym_id="$(config gym_env_id)", space="action"
        )

        # Set observation rates
        spec.sensors.observation.rate = "$(config gym_rate)"
        spec.sensors.reward.rate = "$(config gym_rate)"
        spec.sensors.done.rate = "$(config gym_rate)"
        spec.sensors.image.rate = "$(config gym_rate)"
        spec.actuators.action.rate = "$(config gym_rate)"

    @staticmethod
    @register.spec("GymObject", Object)
    def spec(
        spec: ObjectSpec,
        name: str,
        sensors: Optional[List[str]] = None,
        gym_env_id: str = "Pendulum-v1",
        gym_rate: float = 20.0,
        gym_always_render: bool = False,
        default_action=None,
        render_shape: Optional[List[int]] = None,
    ):
        """Object spec of GymObject"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(GymObject)

        # Set default node params
        spec.config.name = name
        spec.config.sensors = sensors if sensors else ["observation", "reward", "done"]
        spec.config.actuators = ["action"]

        # Set custom node params
        spec.config.render_shape = render_shape if render_shape else [200, 200]
        spec.config.gym_env_id = gym_env_id
        spec.config.gym_rate = gym_rate
        spec.config.gym_always_render = gym_always_render

        # Add bridge implementation
        GymObject.openai_gym(spec)

    @classmethod
    @register.bridge(GymBridge)  # This decorator pre-initializes bridge implementation with default object_params
    def openai_gym(cls, spec: SpecificSpec, graph: EngineGraph):
        """Engine-specific implementation (GymBridge) of the object."""
        # Set bridge arguments (nothing to set here in this case)
        spec.config.env_id = "$(config gym_env_id)"

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        obs = EngineNode.make("ObservationSensor", "obs", rate=None, process=2)
        rwd = EngineNode.make("RewardSensor", "rwd", rate=None, process=2)
        done = EngineNode.make("DoneSensor", "done", rate=None, process=2)
        image = EngineNode.make(
            "GymImage",
            "image",
            shape="$(config render_shape)",
            always_render="$(config gym_always_render)",
            rate=None,
            process=2,
        )

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        action = EngineNode.make(
            "ActionActuator",
            "action",
            rate=None,
            process=2,
            zero_action="$(config default_action)",
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

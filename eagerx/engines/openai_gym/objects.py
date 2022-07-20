# OTHER IMPORTS
from typing import Optional, List

# ROS IMPORTS
import gym

# EAGERx IMPORTS
from eagerx.core.space import Space
from eagerx.engines.openai_gym.engine import GymEngine
from eagerx.core.entities import Object
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class GymObject(Object):
    @classmethod
    @register.sensors(
        observation=Space(dtype="float32"),
        reward=Space(dtype="float32"),
        done=Space(low=0, high=1, shape=(), dtype="int64"),
        image=Space(dtype="uint8"),
    )
    @register.actuators(action=Space(dtype="float32"))
    @register.engine_states()
    def make(
        cls,
        name: str,
        sensors: Optional[List[str]] = None,
        env_id: str = "Pendulum-v1",
        rate: float = 20.0,
        always_render: bool = False,
        default_action=None,
        render_shape: Optional[List[int]] = None,
    ):
        """Object spec of GymObject"""
        spec = cls.get_specification()

        # Set default node params
        spec.config.name = name
        spec.config.sensors = sensors if isinstance(sensors, list) else ["observation", "reward", "done"]
        spec.config.actuators = ["action"]

        # Set custom node params
        spec.config.render_shape = render_shape if render_shape else [200, 200]
        spec.config.env_id = env_id
        spec.config.always_render = always_render
        spec.config.default_action = default_action

        # Set spaces
        env = gym.make(spec.config.env_id)
        obs_space = env.observation_space
        if isinstance(obs_space, gym.spaces.Discrete):
            spec.sensors.observation.space = Space(low=0, high=obs_space.n - 1, shape=(), dtype="int64")
        elif isinstance(obs_space, gym.spaces.Box):
            spec.sensors.observation.space = Space(obs_space.low, obs_space.high, shape=obs_space.shape, dtype=obs_space.dtype)
        else:
            raise NotImplementedError("Space not compatible with this object.")
        act_space = env.action_space
        if isinstance(act_space, gym.spaces.Discrete):
            spec.actuators.action.space = Space(low=0, high=act_space.n - 1, shape=(), dtype="int64")
        elif isinstance(act_space, gym.spaces.Box):
            spec.actuators.action.space = Space(act_space.low, act_space.high, shape=act_space.shape, dtype=act_space.dtype)
        else:
            raise NotImplementedError("Space not compatible with this object.")

        spec.sensors.reward.space = Space(low=env.reward_range[0], high=env.reward_range[1], shape=tuple(), dtype="float32")
        shape = (spec.config.render_shape[0], spec.config.render_shape[1], 3)
        spec.sensors.image.space = Space(low=0, high=255, shape=shape, dtype="uint8")

        # Set rates
        spec.sensors.observation.rate = rate
        spec.sensors.reward.rate = rate
        spec.sensors.done.rate = rate
        spec.sensors.image.rate = rate
        spec.actuators.action.rate = rate
        return spec

    @staticmethod
    @register.engine(GymEngine)  # This decorator pre-initializes engine implementation with default object_params
    def openai_gym(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (GymEngine) of the object."""
        # Set engine arguments (nothing to set here in this case)
        spec.engine.env_id = spec.config.env_id

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        from eagerx.engines.openai_gym.enginenodes import ObservationSensor, RewardSensor, DoneSensor, ActionActuator, GymImage

        obs = ObservationSensor.make("obs", rate=spec.sensors.observation.rate, process=2)
        obs.outputs.observation.space = spec.sensors.observation.space
        rwd = RewardSensor.make("rwd", rate=spec.sensors.reward.rate, process=2)
        done = DoneSensor.make("done", rate=spec.sensors.done.rate, process=2)
        image = GymImage.make(
            "image",
            shape=spec.config.render_shape,
            always_render=spec.config.always_render,
            rate=spec.sensors.image.rate,
            process=2,
        )

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        action = ActionActuator.make(
            "action", rate=spec.actuators.action.rate, process=2, zero_action=spec.config.default_action
        )
        action.inputs.action.space = spec.actuators.action.space
        action.outputs.action_applied.space = spec.actuators.action.space

        # Connect all engine nodes
        graph.add([obs, rwd, done, image, action])
        graph.connect(source=obs.outputs.observation, sensor="observation")
        graph.connect(source=rwd.outputs.reward, sensor="reward")
        graph.connect(source=done.outputs.done, sensor="done")
        graph.connect(source=image.outputs.image, sensor="image")
        graph.connect(actuator="action", target=action.inputs.action)

        # Check graph validity (comment out)
        # graph.is_valid(plot=True)

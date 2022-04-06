from typing import Optional, List
import skimage.transform
import numpy as np
import gym

# IMPORT ROS
import rospy
from std_msgs.msg import UInt64, Float32MultiArray, Bool, Float32
from sensor_msgs.msg import Image

# IMPORT EAGERX
import eagerx.core.register as register
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode
from eagerx.core.constants import process


class ObservationSensor(EngineNode):
    @staticmethod
    @register.spec("ObservationSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """ObservationSensor spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(ObservationSensor)

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["observation"]

    def initialize(self):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.id = self.bridge_config["env_id"]
        self.obj_name = self.config["name"]
        self.last_obs = None

    @register.states()
    def reset(self):
        self.last_obs = None

    @register.inputs(tick=UInt64)
    @register.outputs(observation=Float32MultiArray)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )
        obs = self.simulator[self.obj_name]["buffer_obs"]
        self.simulator[self.obj_name]["buffer_obs"] = []
        if len(obs) == 0:
            obs = self.last_obs
        else:
            obs = obs[-1]
            self.last_obs = obs
        return dict(observation=Float32MultiArray(data=obs))


class RewardSensor(EngineNode):
    @staticmethod
    @register.spec("RewardSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """RewardSensor spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(RewardSensor)

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["reward"]

    def initialize(self):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.id = self.bridge_config["env_id"]
        self.obj_name = self.config["name"]
        self.last_reward = None

    @register.states()
    def reset(self):
        self.last_reward = 0.0

    @register.inputs(tick=UInt64)
    @register.outputs(reward=Float32)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )
        reward = self.simulator[self.obj_name]["buffer_reward"]
        self.simulator[self.obj_name]["buffer_reward"] = []
        if len(reward) == 0:
            reward = self.last_reward
        else:
            self.last_reward = reward[-1]
            reward = sum(reward)
        return dict(reward=Float32(data=reward))


class DoneSensor(EngineNode):
    @staticmethod
    @register.spec("DoneSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """DoneSensor spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(DoneSensor)

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["done"]

    def initialize(self):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.id = self.bridge_config["env_id"]
        self.obj_name = self.config["name"]
        self.last_done = None

    @register.states()
    def reset(self):
        self.last_done = False

    @register.inputs(tick=UInt64)
    @register.outputs(done=Bool)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )
        done = self.simulator[self.obj_name]["buffer_done"]
        self.simulator[self.obj_name]["buffer_done"] = []
        if len(done) == 0:
            done = self.last_done
        else:
            done = any(done) or self.last_done
            self.last_done = done
        return dict(done=Bool(data=done))


class ActionActuator(EngineNode):
    @staticmethod
    @register.spec("ActionActuator", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        zero_action=None,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "green",
    ):
        """ActionActuator spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(ActionActuator)

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick", "action"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["action_applied"]

        # Modify custom node params
        spec.config.zero_action = zero_action

    def initialize(self, zero_action):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.obj_name = self.config["name"]
        self.simulator[self.obj_name]["env"]: gym.Env
        self.is_discrete = (
            True if isinstance(self.simulator[self.obj_name]["env"].action_space, gym.spaces.Discrete) else False
        )
        if zero_action is None:
            self.zero_action = self.simulator[self.obj_name]["env"].action_space.sample()
        else:
            if isinstance(zero_action, list):
                dtype = self.simulator[self.obj_name]["env"].action_space.dtype
                self.zero_action = np.array(zero_action, dtype=dtype)
            else:
                self.zero_action = zero_action
            assert self.simulator[self.obj_name]["env"].action_space.contains(
                self.zero_action
            ), f'The zero action provided for "{self.obj_name}" is not contained in the action space of this environment.'

    @register.states()
    def reset(self):
        # This controller is stateless (in contrast to e.g. a PID controller).
        self.simulator[self.obj_name]["next_action"] = self.zero_action

    @register.inputs(tick=UInt64, action=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action: Optional[Msg] = None,
    ):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )

        # Set action in simulator for next step.
        if len(action.msgs) > 0:
            self.simulator[self.obj_name]["next_action"] = (
                int(action.msgs[-1].data[0]) if self.is_discrete else action.msgs[-1].data
            )
        else:
            self.simulator[self.obj_name]["next_action"] = self.zero_action

        # Prepare output message
        action_applied = self.simulator[self.obj_name]["next_action"]

        # Send action that has been applied.
        return dict(action_applied=Float32MultiArray(data=action_applied))


class GymImage(EngineNode):
    @staticmethod
    @register.spec("GymImage", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.BRIDGE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
        shape: Optional[List[int]] = None,
        always_render=False,
    ):
        """GymImage spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(GymImage)

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["image"]

        # Modify custom node params
        spec.config.shape = shape if isinstance(shape, list) else [200, 200]
        spec.config.always_render = always_render

    def initialize(self, shape, always_render):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.BRIDGE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
        self.shape = tuple(shape)
        self.always_render = always_render
        self.render_toggle = False
        self.id = self.bridge_config["env_id"]
        self.obj_name = self.config["name"]
        self.render_toggle_pub = rospy.Subscriber("%s/env/render/toggle" % self.ns, Bool, self._set_render_toggle)

    @register.states()
    def reset(self):
        # This sensor is stateless (in contrast to e.g. a Kalman filter).
        pass

    @register.inputs(tick=UInt64)
    @register.outputs(image=Image)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )
        if self.always_render or self.render_toggle:
            rgb = self.simulator[self.obj_name]["env"].render(mode="rgb_array")

            # Resize image if not matching desired self.shape (defined in .yaml)
            if rgb.shape[:2] != tuple(self.shape):
                kwargs = dict(output_shape=self.shape, mode="edge", order=1, preserve_range=True)
                rgb = skimage.transform.resize(rgb, **kwargs).astype(rgb.dtype)

            # Prepare ROS msg
            height = rgb.shape[0]
            width = rgb.shape[1]
            data = rgb.tobytes("C")
            msg = Image(data=data, height=height, width=width, encoding="rgb8")
            # self._show_ros_image(msg)
        else:
            msg = Image()
        return dict(image=msg)

    def _show_ros_image(self, msg):
        import matplotlib.pyplot as plt

        rgb_back = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        plt.imshow(rgb_back)
        plt.show()

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo("[%s] START RENDERING!" % self.name)
        else:
            self.simulator[self.obj_name]["env"].close()
            rospy.loginfo("[%s] STOPPED RENDERING!" % self.name)
        self.render_toggle = msg.data

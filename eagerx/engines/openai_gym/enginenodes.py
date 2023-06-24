from typing import Optional, List
import PIL
import numpy as np
import gymnasium as gym
from pyvirtualdisplay import Display
import os
import threading

# IMPORT EAGERX
import eagerx.core.register as register
from eagerx.core.space import Space
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode
from eagerx.core.constants import process


class ObservationSensor(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """ObservationSensor spec"""
        spec = cls.get_specification()

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["observation"]
        return spec

    def initialize(self, spec, simulator):
        # We will probably use self.simulator in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.simulator = simulator
        self.id = simulator["env_id"]
        self.last_obs = None

    @register.states()
    def reset(self):
        self.last_obs = None

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(observation=Space(dtype="float32"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator, dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator
        )
        obs = self.simulator["buffer_obs"]
        self.simulator["buffer_obs"] = []
        obs = self.last_obs if len(obs) == 0 else obs[-1]
        self.last_obs = obs
        return dict(observation=obs)


class RewardSensor(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """RewardSensor spec"""
        spec = cls.get_specification()

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["reward"]
        return spec

    def initialize(self, spec, simulator):
        # We will probably use self.simulator in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.simulator = simulator
        self.id = simulator["env_id"]
        self.last_reward = None

    @register.states()
    def reset(self):
        self.last_reward = 0.0

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(reward=Space(dtype="float32"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator, dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator
        )
        reward = self.simulator["buffer_reward"]
        self.simulator["buffer_reward"] = []
        if len(reward) == 0:
            reward = self.last_reward
        else:
            self.last_reward = reward[-1]
            reward = sum(reward)
        return dict(reward=reward)


class TruncatedSensor(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """DoneSensor spec"""
        spec = cls.get_specification()

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["truncated"]
        return spec

    def initialize(self, spec, simulator):
        # We will probably use self.simulator in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.simulator = simulator
        self.id = simulator["env_id"]
        self.last_truncated = None

    @register.states()
    def reset(self):
        self.last_truncated = False

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(truncated=Space(low=0, high=1, shape=(), dtype="int64"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator, dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator
        )
        truncated = self.simulator["buffer_truncated"]
        self.simulator["buffer_truncated"] = []
        if len(truncated) == 0:
            truncated = self.last_truncated
        else:
            truncated = any(truncated) or self.last_truncated
            self.last_truncated = truncated
        return dict(truncated=int(truncated))


class TerminatedSensor(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """DoneSensor spec"""
        spec = cls.get_specification()

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["terminated"]
        return spec

    def initialize(self, spec, simulator):
        # We will probably use self.simulator in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.simulator = simulator
        self.id = simulator["env_id"]
        self.last_terminated = None

    @register.states()
    def reset(self):
        self.last_terminated = False

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(terminated=Space(low=0, high=1, shape=(), dtype="int64"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator, dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator
        )
        terminated = self.simulator["buffer_terminated"]
        self.simulator["buffer_terminated"] = []
        if len(terminated) == 0:
            terminated = self.last_terminated
        else:
            terminated = any(terminated) or self.last_terminated
            self.last_terminated = terminated
        return dict(terminated=int(terminated))


class ActionActuator(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        zero_action=None,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "green",
        delay_state: bool = False,
    ):
        """ActionActuator spec"""
        spec = cls.get_specification()

        # Set default
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick", "action"]
        spec.config.outputs = outputs if isinstance(outputs, list) else ["action_applied"]
        spec.config.states = ["delay"] if delay_state else []

        # Modify custom node params
        spec.config.zero_action = zero_action
        return spec

    def initialize(self, spec, simulator):
        # We will probably use self.simulator in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.simulator = simulator
        self.simulator["env"]: gym.Env
        self.is_discrete = True if isinstance(self.simulator["env"].action_space, gym.spaces.Discrete) else False
        if spec.config.zero_action is None:
            self.zero_action = self.simulator["env"].action_space.sample()
        else:
            if isinstance(spec.config.zero_action, list):
                dtype = self.simulator["env"].action_space.dtype
                self.zero_action = np.array(spec.config.zero_action, dtype=dtype)
            else:
                self.zero_action = spec.config.zero_action
            assert self.simulator["env"].action_space.contains(
                self.zero_action
            ), f'The zero action provided for "{self.simulator["name"]}" is not contained in the action space of this environment.'

    @register.states(delay=Space(shape=(), low=0.0, high=0.0, dtype="float32"))
    def reset(self, delay=None):
        # This controller is stateless (in contrast to e.g. a PID controller).
        if delay is not None:
            self.set_delay(delay, "inputs", "action")
        self.simulator["next_action"] = self.zero_action

    @register.inputs(tick=Space(shape=(), dtype="int64"), action=Space(dtype="float32"))
    @register.outputs(action_applied=Space(dtype="float32"))
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action: Optional[Msg] = None,
    ):
        assert isinstance(self.simulator, dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator
        )

        # Set action in simulator for next step.
        if len(action.msgs) > 0:
            self.simulator["next_action"] = int(action.msgs[-1].item()) if self.is_discrete else action.msgs[-1]
        else:
            self.simulator["next_action"] = self.zero_action

        # Prepare output message
        action_applied = self.simulator["next_action"]

        # Send action that has been applied.
        return dict(action_applied=action_applied)


class GymImage(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
        shape: Optional[List[int]] = None,
        always_render=False,
    ):
        """GymImage spec"""
        spec = cls.get_specification()

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
        spec.outputs.image.space = Space(low=0, high=255, shape=tuple(spec.config.shape + [3]), dtype="uint8")
        return spec

    def initialize(self, spec, simulator):
        # We will probably use self.simulator in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.simulator = simulator
        self.shape = tuple(spec.config.shape)
        self.always_render = spec.config.always_render
        self.render_toggle = False
        self.id = simulator["env_id"]
        self._cond = threading.Condition()
        self.sub_toggle = self.backend.Subscriber("%s/env/render/toggle" % self.ns, "bool", self._set_render_toggle)

        # Setup virtual display for rendering.
        self.display = Display(visible=False, backend="xvfb")
        self.disp_id = os.environ["DISPLAY"]  # First record default display id
        self.display.start()
        self.xvfb_id = self.display.env()["DISPLAY"]  # Get virtual display id
        os.environ["DISPLAY"] = self.disp_id  # Reset to default display id

    @register.states()
    def reset(self):
        # This sensor is stateless (in contrast to e.g. a Kalman filter).
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(image=Space(dtype="uint8"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator, dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator
        )
        with self._cond:
            if self.always_render or self.render_toggle:
                os.environ["DISPLAY"] = self.xvfb_id  # Set virtual display id
                try:
                    rgb = self.simulator["env"].render()
                except Exception as e:
                    self.backend.logwarn(e)
                    raise e
                os.environ["DISPLAY"] = self.disp_id  # Reset to default display id

                # Resize image if not matching desired self.shape
                if rgb.shape[:2] != tuple(self.shape):
                    img_PIL = PIL.Image.fromarray(rgb).convert("RGB")
                    img_PIL = img_PIL.resize(self.shape, PIL.Image.ANTIALIAS)
                    rgb = np.array(img_PIL).astype(rgb.dtype)
            else:
                rgb = np.zeros((self.shape[0], self.shape[1], 3), np.uint8)
        return dict(image=rgb)

    def shutdown(self):
        self.backend.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()

    def _show_ros_image(self, msg):
        import matplotlib.pyplot as plt

        rgb_back = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        plt.imshow(rgb_back)
        plt.show()

    def _set_render_toggle(self, msg):
        with self._cond:
            if msg:
                self.backend.logdebug("[%s] START RENDERING!" % self.name)
            else:
                self.simulator["env"].close()
                self.backend.logdebug("[%s] STOPPED RENDERING!" % self.name)
            self.render_toggle = msg

from typing import Optional, List

import pyglet.gl.lib
import skimage.transform
import numpy as np
import gym
from pyvirtualdisplay import Display
import os
import threading
from gym.spaces import Discrete, Box

# IMPORT EAGERX
import eagerx.core.ros1 as bnd
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
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """ObservationSensor spec"""
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
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.id = self.engine_config["env_id"]
        self.obj_name = self.config["name"]
        self.last_obs = None

    @register.states()
    def reset(self):
        self.last_obs = None

    @register.inputs(tick=Discrete(99999))
    @register.outputs(observation=None)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )
        obs = self.simulator[self.obj_name]["buffer_obs"]
        self.simulator[self.obj_name]["buffer_obs"] = []
        obs = self.last_obs if len(obs) == 0 else obs[-1]
        self.last_obs = obs
        return dict(observation=obs)


class RewardSensor(EngineNode):
    @staticmethod
    @register.spec("RewardSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """RewardSensor spec"""
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
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.id = self.engine_config["env_id"]
        self.obj_name = self.config["name"]
        self.last_reward = None

    @register.states()
    def reset(self):
        self.last_reward = 0.0

    @register.inputs(tick=Discrete(99999))
    @register.outputs(reward=None)
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
        return dict(reward=reward)


class DoneSensor(EngineNode):
    @staticmethod
    @register.spec("DoneSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "cyan",
    ):
        """DoneSensor spec"""
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
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.id = self.engine_config["env_id"]
        self.obj_name = self.config["name"]
        self.last_done = None

    @register.states()
    def reset(self):
        self.last_done = False

    @register.inputs(tick=Discrete(99999))
    @register.outputs(done=Discrete(2))
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
        return dict(done=int(done))


class ActionActuator(EngineNode):
    @staticmethod
    @register.spec("ActionActuator", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        zero_action=None,
        process: Optional[int] = process.ENGINE,
        inputs: Optional[List[str]] = None,
        outputs: Optional[List[str]] = None,
        color: Optional[str] = "green",
    ):
        """ActionActuator spec"""
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
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
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

    @register.inputs(tick=Discrete(99999), action=None)
    @register.outputs(action_applied=None)
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
            self.simulator[self.obj_name]["next_action"] = int(action.msgs[-1].item()) if self.is_discrete else action.msgs[-1]
        else:
            self.simulator[self.obj_name]["next_action"] = self.zero_action

        # Prepare output message
        action_applied = self.simulator[self.obj_name]["next_action"]

        # Send action that has been applied.
        return dict(action_applied=action_applied)


class GymImage(EngineNode):
    @staticmethod
    @register.spec("GymImage", EngineNode)
    def spec(
        spec,
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
        spec.outputs.image.space = Box(low=0, high=255, shape=spec.config.shape + [3], dtype="uint8")

    def initialize(self, shape, always_render):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert (
            self.process == process.ENGINE
        ), "Simulation node requires a reference to the simulator, hence it must be launched in the Engine process"
        self.shape = tuple(shape)
        self.always_render = always_render
        self.render_toggle = False
        self.id = self.engine_config["env_id"]
        self.obj_name = self.config["name"]
        self.sub_toggle = bnd.Subscriber("%s/env/render/toggle" % self.ns, "bool", self._set_render_toggle)
        self._cond = threading.Condition()

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

    @register.inputs(tick=Discrete(99999))
    @register.outputs(image=None)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        assert isinstance(self.simulator[self.obj_name], dict), (
            'Simulator object "%s" is not compatible with this engine node.' % self.simulator[self.obj_name]
        )
        with self._cond:
            if self.always_render or self.render_toggle:
                os.environ["DISPLAY"] = self.xvfb_id  # Set virtual display id
                try:
                    rgb = self.simulator[self.obj_name]["env"].render(mode="rgb_array")
                except pyglet.gl.lib.GLException as e:
                    bnd.logwarn(e)
                    raise e
                os.environ["DISPLAY"] = self.disp_id  # Reset to default display id

                # Resize image if not matching desired self.shape
                if rgb.shape[:2] != tuple(self.shape):
                    kwargs = dict(output_shape=self.shape, mode="edge", order=1, preserve_range=True)
                    rgb = skimage.transform.resize(rgb, **kwargs).astype(rgb.dtype)
            else:
                rgb = np.zeros((self.shape[0], self.shape[1], 3), np.uint8)
        return dict(image=rgb)

    def shutdown(self):
        bnd.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()

    def _show_ros_image(self, msg):
        import matplotlib.pyplot as plt

        rgb_back = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        plt.imshow(rgb_back)
        plt.show()

    def _set_render_toggle(self, msg):
        with self._cond:
            if msg:
                bnd.logdebug("[%s] START RENDERING!" % self.name)
            else:
                self.simulator[self.obj_name]["env"].close()
                bnd.logdebug("[%s] STOPPED RENDERING!" % self.name)
            self.render_toggle = msg

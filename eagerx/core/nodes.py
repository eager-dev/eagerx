import ctypes
import multiprocessing
from typing import Optional, List, Tuple
from threading import Event
from multiprocessing.sharedctypes import RawArray, Value
from multiprocessing import Event as MpEvent
import signal

import time
import sys

import numpy as np
import cv2

import eagerx
import eagerx.core.register as register
from eagerx.core.specs import NodeSpec
from eagerx.utils.utils import initialize_processor, Msg


class EnvNode(eagerx.Node):
    @classmethod
    def make(cls, rate=1, log_level=eagerx.log.WARN, color="yellow"):
        """EnvNode Spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = "environment"
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = []
        spec.config.outputs = []
        spec.config.states = []
        return spec

    def initialize(self, spec: NodeSpec):
        # Define observation buffers
        self.observation_buffer = dict()
        for cname, i in self.inputs.items():
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])
            assert i["space"] is not None, f"No space defined for observation {cname}."
            assert i[
                "space"
            ].is_fully_defined, f"The space for observation {cname} is not fully defined (low, high, shape, dtype)."
            window = i["window"]
            self.observation_buffer[cname] = {
                "msgs": None,
                "processor": i["processor"],
                "window": window,
                "space": i["space"],
            }

        # Synchronization event
        self.obs_event = Event()
        self.action_event = Event()
        self.must_reset = False

        # Graceful signal handler
        def signal_handler(sig, frame):
            self.backend.logdebug("SIGINT caught!")
            self.obs_event.set()
            self.action_event.set()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        # Define action buffers
        self.action_buffer = dict()
        for cname, i in self.outputs.items():
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])
            assert i["space"] is not None, f"No space defined for action {cname}."
            assert i["space"].is_fully_defined, f"The space for action {cname} is not fully defined (low, high, shape, dtype)."

            self.action_buffer[cname] = {"msg": None, "processor": i["processor"], "space": i["space"]}

    def reset(self):
        self.must_reset = False

        # Set all observation messages to None
        for _name, buffer in self.observation_buffer.items():
            buffer["msgs"] = None

        # Set all action messages to None
        for _name, buffer in self.action_buffer.items():
            buffer["msg"] = None

    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        for name, i in kwargs.items():
            buffer = self.observation_buffer[name]
            window = buffer["window"]

            extra = window - len(i.msgs)
            if extra > 0:
                # Only happens when skip=True && window > 0
                if len(i.msgs) == 0:
                    initial_obs = np.empty(buffer["space"].shape)
                    initial_obs[:] = np.NaN
                    i.msgs.append(initial_obs)
                    extra -= 1  # Subtract, because we appended the initial_obs
                msgs = extra * [i.msgs[0]] + i.msgs
            else:
                msgs = i.msgs
            buffer["msgs"] = np.array(msgs)

        if not self.must_reset:
            self.action_event.clear()  # Clear action event, so that we can block after setting obs
            self.obs_event.set()  # Signal environment that observations have been set.
            try:
                # flag = self.action_event.wait(5)  # Wait for actions to be set.
                flag = self.action_event.wait()  # Wait for actions to be set.
                if not flag:
                    raise KeyboardInterrupt
            except (KeyboardInterrupt, SystemExit):
                self.backend.logdebug(f"[{self.ns_name}] KEYBOARD INTERRUPT")
                raise

        if self.must_reset:
            return None
        else:
            # Fill output_msg with buffered actions
            output_msgs = dict()
            for name, buffer in self.action_buffer.items():
                output_msgs[name] = buffer["msg"]
            return output_msgs

    def shutdown(self):
        self.obs_event.set()


class ObservationsNode(eagerx.Node):
    @classmethod
    def make(cls, rate=1, log_level=eagerx.log.WARN, color="yellow"):
        """ObservationsNode spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = "env/observations"
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = ["actions_set"]
        spec.config.outputs = ["set"]
        spec.config.states = []

        # Pre-set address to mock an actual input
        spec.inputs.actions_set.address = "env/actions/outputs/set"
        return spec

    def initialize(self, spec: NodeSpec):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    def reset(self):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    @register.inputs(actions_set=eagerx.Space(dtype="int64"))
    @register.outputs(set=eagerx.Space(dtype="int64"))
    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")


class ActionsNode(eagerx.Node):
    @classmethod
    def make(cls, rate=1, log_level=eagerx.log.WARN, color="yellow"):
        """ActionsNode spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = "env/actions"
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = ["step"]
        spec.config.outputs = ["set"]
        spec.config.states = []

        # Pre-set address to mock an actual input
        spec.inputs.step.address = "env/supervisor/outputs/step"
        return spec

    def initialize(self, spec: NodeSpec):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    def reset(self):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    @register.inputs(observations_set=eagerx.Space(dtype="int64"), step=eagerx.Space(dtype="int64"))
    @register.outputs(set=eagerx.Space(dtype="int64"))
    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")


class RenderNode(eagerx.Node):
    @classmethod
    def make(
        cls,
        rate,
        display=True,
        log_level=eagerx.log.WARN,
        color="grey",
        process=eagerx.process.NEW_PROCESS,
        encoding="bgr",
    ):
        """RenderNode spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = "env/render"
        spec.config.rate = rate
        assert process == eagerx.process.NEW_PROCESS, "Render requires process=NEW_PROCESS, because of cv2 limitations."
        spec.config.process = process
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = ["image"]
        spec.config.outputs = ["done"]
        spec.config.states = []

        # Modify custom params
        spec.config.display = display
        spec.config.encoding = encoding

        # Pre-set window
        spec.inputs.image.window = 1
        return spec

    def initialize(self, spec):
        self.display = spec.config.display
        self.window = None
        self.encoding = spec.config.encoding
        self.last_image = np.empty(shape=(0, 0, 3), dtype="uint8")
        self.render_toggle = Value("i", False)
        self.sub_toggle = self.backend.Subscriber("%s/%s/toggle" % (self.ns, self.name), "bool", self._set_render_toggle)
        self.sub_get = self.backend.Subscriber("%s/%s/get_last_image" % (self.ns, self.name), "bool", self._get_last_image)
        self.pub_set = self.backend.Publisher("%s/%s/set_last_image" % (self.ns, self.name), "uint8")

        # Setup async imshow (opening, closing, and imshow must all be in the same thread).
        assert hasattr(spec.inputs.image.space, "shape"), (
            "The node's outputs that is connected to " "the render node must have a space with shape defined."
        )
        self.img_event = MpEvent()
        self.p = None  # Set in _start_render_process(..)
        self.shared_array_np = None  # Set in _start_render_process(..)

    def _start_render_process(self, shape):
        size = None
        for i in shape:
            if size is None:
                size = i
            else:
                size *= i
        self.shared_array_np = np.ndarray(shape, dtype="uint8", buffer=RawArray(ctypes.c_uint8, size))
        args = (self.ns_name, shape, self.shared_array_np, self.img_event, self.render_toggle)
        self.p = multiprocessing.Process(target=self._async_imshow, args=args)
        self.p.start()

    @staticmethod
    def _async_imshow(ns_name: str, shape: Tuple[int], shared_array: np.ndarray, img_event: Value, render_toggle: Value):
        img = shared_array.view(dtype="uint8").reshape(*shape)
        try:
            while True:
                img_event.wait()  # Wait for event (ie new image or close window)
                img_event.clear()  # Clear event
                if render_toggle.value:
                    cv2.imshow(f"{ns_name}", img)
                cv2.waitKey(1)
        except (KeyboardInterrupt, SystemExit):
            # print(f"[{self.ns_name}]: KeyboardInterrupt caught")
            return

    def _set_render_toggle(self, msg):
        if msg:
            self.backend.logdebug("START RENDERING!")
        else:
            self.backend.logdebug("STOP RENDERING!")
        self.render_toggle.value = msg

    def _get_last_image(self, msg):
        self.pub_set.publish(self.last_image)

    def reset(self):
        pass

    @register.inputs(image=eagerx.Space(dtype="uint8"))
    @register.outputs(done=eagerx.Space(low=0, high=1, shape=(), dtype="int64"))
    def callback(self, t_n: float, image: Optional[Msg] = None):
        self.last_image = image.msgs[-1] if self.encoding == "rgb" else cv2.cvtColor(image.msgs[-1], cv2.COLOR_BGR2RGB)
        empty = len(image.msgs[-1]) == 0
        if not empty and self.display and self.render_toggle.value:
            img = image.msgs[-1] if self.encoding == "bgr" else cv2.cvtColor(image.msgs[-1], cv2.COLOR_RGB2BGR)
            if self.p is None:
                self._start_render_process(img.shape)  # Start rendering in separate process
            np.copyto(self.shared_array_np, img)
        self.img_event.set()  # Signal async_imshow thread
        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=1)
        return output_msgs

    def shutdown(self):
        self.backend.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()
        self.sub_get.unregister()
        self.pub_set.unregister()

        # Close render window
        if self.p is not None:
            self.p.kill()
        self.backend.logdebug(f"[{self.name}] {self.name}.shutdown() done.")


class ColabRender(eagerx.Node):
    @classmethod
    def make(
        cls,
        rate: int,
        process: int = eagerx.process.ENVIRONMENT,
        fps: int = 25,
        shape: Optional[List[int]] = None,
        maxlen: int = 200,
        subsample: bool = True,
        log_level: int = eagerx.log.WARN,
        color: str = "grey",
        encoding="bgr",
    ):
        """ColabRender spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.name = "env/render"
        spec.config.rate = rate
        assert process == eagerx.process.ENVIRONMENT, "ColabRendering must happen in the same process as the environment."
        spec.config.process = process
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = ["image"]
        spec.config.outputs = ["done"]
        spec.config.states = []

        # Custom params
        spec.config.encoding = encoding
        spec.config.fps = fps
        spec.config.shape = shape if isinstance(shape, list) else [64, 64]
        spec.config.maxlen = maxlen
        spec.config.subsample = True

        # Pre-set window
        spec.inputs.image.window = 1
        return spec

    def initialize(self, spec):
        # todo: Overwrite fps if higher than rate
        # todo: Subsample if fps lower than rate * real_time_factor
        # todo: set node_fps either slightly higher or lower than js_fps?
        # todo: Avoid overflowing buffer
        try:
            from eagerx.core.colab_render import InlineRender

            # Store cls as attribute so that it can be initialized in the callback
            self.window_cls = InlineRender
        except ImportError as e:
            self.backend.logerr(f"{e}. This node `ColabRender` can only be used in google colab.")
            raise
        self.dt_fps = 1 / spec.config.fps
        self.subsample = spec.config.subsample
        self.fps = spec.config.fps
        self.shape = spec.config.shape
        self.maxlen = spec.config.maxlen
        self.window = None
        self.encoding = spec.config.encoding
        self.last_image = np.empty(shape=(0, 0, 3), dtype="uint8")
        self.render_toggle = False
        self.sub_toggle = self.backend.Subscriber("%s/%s/toggle" % (self.ns, self.name), "bool", self._set_render_toggle)
        self.sub_get = self.backend.Subscriber("%s/%s/get_last_image" % (self.ns, self.name), "bool", self._get_last_image)
        self.pub_set = self.backend.Publisher("%s/%s/set_last_image" % (self.ns, self.name), "uint8")

    def _set_render_toggle(self, msg):
        if msg:
            self.backend.logdebug("START RENDERING!")
        else:
            self.backend.logdebug("STOP RENDERING!")
        self.render_toggle = msg

    def _get_last_image(self, msg):
        self.pub_set.publish(self.last_image)

    def reset(self):
        self.last_image = np.empty(shape=(0, 0, 3), dtype="uint8")

    @register.inputs(image=None)
    @register.outputs(done=eagerx.Space(low=0, high=1, shape=(), dtype="int64"))
    def callback(self, t_n: float, image: Optional[eagerx.utils.utils.Msg] = None):
        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=0)
        # Grab latest image
        self.last_image = image.msgs[-1] if self.encoding == "rgb" else cv2.cvtColor(image.msgs[-1], cv2.COLOR_BGR2RGB)
        # If too little time has passed, do not add frame (avoid buffer overflowing)
        if self.window is None:
            self.window = self.window_cls(fps=self.fps, maxlen=self.maxlen, shape=self.shape)
        elif not time.time() > (self.dt_fps + self.window.timestamp):
            return output_msgs
        # Check if frame is not empty
        empty = len(self.last_image.data) == 0
        if not empty and self.render_toggle:
            # Convert to rgb (from bgr)
            img = image.msgs[-1] if self.encoding == "bgr" else cv2.cvtColor(image.msgs[-1], cv2.COLOR_RGB2BGR)
            # Add image to buffer (where it is send async to javascript window)
            self.window.buffer_images(img)
        return output_msgs

    def shutdown(self):
        self.backend.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()
        self.sub_get.unregister()
        self.pub_set.unregister()

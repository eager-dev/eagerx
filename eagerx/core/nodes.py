from typing import Optional, List
from threading import Event, Thread, Condition
import signal
import time
import sys

import numpy as np
from gym.spaces import Discrete
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv_bridge
import cv2

import eagerx
import eagerx.core.register as register
from eagerx.core.specs import NodeSpec
from eagerx.utils.utils import initialize_converter, Msg, dict_to_space


class EnvNode(eagerx.Node):
    @staticmethod
    @register.spec("Environment", eagerx.Node)
    def spec(spec: NodeSpec, rate=1, log_level=eagerx.log.WARN, color="yellow"):
        """EnvNode Spec"""
        # Modify default node params
        spec.config.name = "environment"
        spec.config.rate = rate
        spec.config.process = eagerx.process.ENVIRONMENT
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = []
        spec.config.outputs = []
        spec.config.states = []

    def initialize(self):
        # Define observation buffers
        self.observation_buffer = dict()
        for i in self.inputs:
            if i["name"] == "actions_set":
                continue
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_converter(i["processor"])
                processor = i["processor"]
            else:
                processor = i["processor"]
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])
                space = i["space"]
            else:
                space = i["space"]
            assert space is not None, f'No space defined for observation {i["name"]}.'

            name = i["name"]
            window = i["window"]
            self.observation_buffer[name] = {
                "msgs": None,
                "processor": processor,
                "window": window,
                "space": space,
            }

        # Synchronization event
        self.obs_event = Event()
        self.action_event = Event()
        self.must_reset = False

        # Graceful signal handler
        def signal_handler(sig, frame):
            print("SIGINT caught!")
            self.obs_event.set()
            self.action_event.set()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        # Define action buffers
        self.action_buffer = dict()
        for i in self.outputs:
            if i["name"] == "set":
                continue
            if isinstance(i["processor"], dict):
                i["processor"] = initialize_converter(i["processor"])
                processor = i["processor"]
            else:
                processor = i["processor"]
            if isinstance(i["space"], dict):
                i["space"] = dict_to_space(i["space"])
                space = i["space"]
            else:
                space = i["space"]
            assert space is not None, f'No space defined for observation {i["name"]}.'

            self.action_buffer[i["name"]] = {"msg": None, "processor": processor, "space": space}

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
                    raise NotImplementedError("initial_obs not yet refactored.")
                    initial_obs = buffer["converter"].initial_obs
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
                print(f"[{self.ns_name}] KEYBOARD INTERRUPT")
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
    @staticmethod
    @register.spec("Observations", eagerx.Node)
    def spec(spec: NodeSpec, rate=1, log_level=eagerx.log.WARN, color="yellow"):
        """ObservationsNode spec"""
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

    def initialize(self):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    def reset(self):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    @register.inputs(actions_set=Discrete(np.iinfo("int32").max))
    @register.outputs(set=Discrete(np.iinfo("int32").max))
    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")


class ActionsNode(eagerx.Node):
    @staticmethod
    @register.spec("Actions", eagerx.Node)
    def spec(spec: NodeSpec, rate=1, log_level=eagerx.log.WARN, color="yellow"):
        """ActionsNode spec"""
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

    def initialize(self):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    def reset(self):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")

    @register.inputs(observations_set=Discrete(np.iinfo("int32").max), step=Discrete(np.iinfo("int32").max))
    @register.outputs(set=Discrete(np.iinfo("int32").max))
    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")


class RenderNode(eagerx.Node):
    @staticmethod
    @register.spec("Render", eagerx.Node)
    def spec(
        spec: NodeSpec,
        rate,
        display=True,
        log_level=eagerx.log.WARN,
        color="grey",
        process=eagerx.process.NEW_PROCESS,
        encoding="bgr",
    ):
        """RenderNode spec"""
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
        spec.inputs.image.window = 0

    def initialize(self, display, encoding):
        self.cv_bridge = cv_bridge.CvBridge()
        self.window = None
        self.display = display
        self.encoding = encoding
        self.last_image = Image(data=[])
        self.render_toggle = False
        self.sub_toggle = rospy.Subscriber("%s/%s/toggle" % (self.ns, self.name), Bool, self._set_render_toggle)
        self.sub_get = rospy.Subscriber("%s/%s/get_last_image" % (self.ns, self.name), Bool, self._get_last_image)
        self.pub_set = rospy.Publisher(
            "%s/%s/set_last_image" % (self.ns, self.name),
            Image,
            queue_size=0,
            latch=True,
        )

        # Setup async imshow (opening, closing, and imshow must all be in the same thread).
        self.window_open = False
        self.must_close = False
        self.stop_thread = False
        self.cv_image = None
        self.img_event = Event()
        self.img_cond = Condition()
        self.img_thread = Thread(target=self._async_imshow, args=())
        self.img_thread.start()

    def _async_imshow(self):
        while True:
            self.img_event.wait()  # Wait for event (ie new image or close window)
            self.img_event.clear()  # Clear event
            if self.stop_thread:
                break
            if self.window_open and self.must_close:  # We must close the window
                cv2.destroyWindow(f"{self.ns_name}")
                self.must_close = False
                self.window_open = False
                continue
            if self.cv_image is None:
                continue
            if self.render_toggle:
                cv2.imshow(f"{self.ns_name}", self.cv_image)
            cv2.waitKey(1)
            self.window_open = True

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo("START RENDERING!")
        else:
            rospy.loginfo("STOP RENDERING!")
        self.render_toggle = msg.data

    def _get_last_image(self, msg):
        self.pub_set.publish(self.last_image)

    def reset(self):
        pass

    @register.inputs(image=None)
    @register.outputs(done=Discrete(2))
    def callback(self, t_n: float, image: Optional[Msg] = None):
        if len(image.msgs) > 0:
            self.last_image = image.msgs[-1]
        else:
            return dict(done=np.array(0, dtype="float32"))
        empty = len(image.msgs[-1]) == 0
        if not empty and self.display and self.render_toggle:
            self.cv_image = image.msgs[-1] if self.encoding == "bgr" else cv2.cvtColor(image.msgs[-1], cv2.COLOR_RGB2BGR)
        with self.img_cond:
            self.img_event.set()  # Signal async_imshow thread

        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=np.array(0, dtype="float32"))
        return output_msgs

    def shutdown(self):
        rospy.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()
        self.sub_get.unregister()
        self.pub_set.unregister()

        # Close render window
        with self.img_cond:
            self.must_close = True
            self.img_event.set()
            # Wait for render window to be closed
            while self.window_open:
                time.sleep(0.01)
            self.stop_thread = True
            self.img_event.set()


class ColabRender(eagerx.Node):
    @staticmethod
    @register.spec("ColabRender", eagerx.Node)
    def spec(
        spec: NodeSpec,
        rate: int,
        process: int = eagerx.process.ENVIRONMENT,
        fps: int = 25,
        shape: Optional[List[int]] = None,
        maxlen: int = 200,
        subsample: bool = True,
        log_level: int = eagerx.log.WARN,
        color: str = "grey",
    ):
        """ColabRender spec"""
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
        spec.config.fps = fps
        spec.config.shape = shape if isinstance(shape, list) else [64, 64]
        spec.config.maxlen = maxlen
        spec.config.subsample = True

        # Pre-set window
        spec.inputs.image.window = 0

    def initialize(self, fps, maxlen, shape, subsample):
        # todo: Overwrite fps if higher than rate
        # todo: Subsample if fps lower than rate * real_time_factor
        # todo: set node_fps either slightly higher or lower than js_fps?
        # todo: Avoid overflowing buffer
        try:
            from eagerx.core.colab_render import InlineRender

            # Store cls as attribute so that it can be initialized in the callback
            self.window_cls = InlineRender
        except ImportError as e:
            rospy.logerr(f"{e}. This node `ColabRender` can only be used in google colab.")
            raise
        self.dt_fps = 1 / fps
        self.subsample = subsample
        self.fps = fps
        self.shape = shape
        self.maxlen = maxlen
        self.window = None
        self.last_image = Image(data=[])
        self.render_toggle = False
        self.sub_toggle = rospy.Subscriber("%s/%s/toggle" % (self.ns, self.name), Bool, self._set_render_toggle)
        self.sub_get = rospy.Subscriber("%s/%s/get_last_image" % (self.ns, self.name), Bool, self._get_last_image)
        self.pub_set = rospy.Publisher(
            "%s/%s/set_last_image" % (self.ns, self.name),
            Image,
            queue_size=0,
            latch=True,
        )

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo("START RENDERING!")
        else:
            rospy.loginfo("STOP RENDERING!")
        self.render_toggle = msg.data

    def _get_last_image(self, msg):
        self.pub_set.publish(self.last_image)

    def reset(self):
        self.last_image = Image(data=[])

    @register.inputs(image=None)
    @register.outputs(done=Discrete(2))
    def callback(self, t_n: float, image: Optional[eagerx.utils.utils.Msg] = None):
        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=0)
        # Grab latest image
        if len(image.msgs) > 0:
            self.last_image = image.msgs[-1]
        # If too little time has passed, do not add frame (avoid buffer overflowing)
        if self.window is None:
            self.window = self.window_cls(fps=self.fps, maxlen=self.maxlen, shape=self.shape)
        elif not time.time() > (self.dt_fps + self.window.timestamp):
            return output_msgs
        # Check if frame is not empty
        empty = self.last_image.height == 0 or self.last_image.width == 0
        if not empty and self.render_toggle:
            # Convert image to np array
            if isinstance(self.last_image.data, bytes):
                img = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(
                    self.last_image.height, self.last_image.width, -1
                )
            else:
                img = np.array(self.last_image.data, dtype=np.uint8).reshape(self.last_image.height, self.last_image.width, -1)
            # Convert to rgb (from bgr)
            if "bgr" in self.last_image.encoding:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Add image to buffer (where it is send async to javascript window)
            self.window.buffer_images(img)
        return output_msgs

    def shutdown(self):
        rospy.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()
        self.sub_get.unregister()
        self.pub_set.unregister()

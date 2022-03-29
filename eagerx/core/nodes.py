from typing import Optional
from threading import Event
import signal
import sys

import numpy as np
import rospy
from std_msgs.msg import UInt64, Bool
from sensor_msgs.msg import Image
import cv_bridge
import cv2


import eagerx.core.register as register
from eagerx.core.constants import process, WARN
from eagerx.core.entities import Node
from eagerx.core.specs import NodeSpec
from eagerx.utils.utils import initialize_converter, Msg


class EnvNode(Node):
    @staticmethod
    @register.spec("Environment", Node)
    def spec(spec: NodeSpec, rate=1, log_level=WARN, color="yellow"):
        """EnvNode Spec"""
        spec.initialize(EnvNode)

        # Modify default node params
        spec.config.name = "environment"
        spec.config.rate = rate
        spec.config.process = process.ENVIRONMENT
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
            if "converter" in i and isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
                converter = i["converter"]
            elif "converter" in i and not isinstance(i["converter"], dict):
                converter = i["converter"]
            else:
                raise ValueError(f'Converter type {i["converter"]} of {i["name"]} not supported.')

            name = i["name"]
            window = i["window"]
            self.observation_buffer[name] = {
                "msgs": None,
                "converter": converter,
                "window": window,
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
            if "converter" in i and isinstance(i["converter"], dict):
                i["converter"] = initialize_converter(i["converter"])
                converter = i["converter"]
            elif "converter" in i and not isinstance(i["converter"], dict):
                converter = i["converter"]
            else:
                converter = None
            self.action_buffer[i["name"]] = {"msg": None, "converter": converter}

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
                print("[env] KEYBOARD INTERRUPT")
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


class ObservationsNode(Node):
    @staticmethod
    @register.spec("Observations", Node)
    def spec(spec: NodeSpec, rate=1, log_level=WARN, color="yellow"):
        """ObservationsNode spec"""
        # Initialize spec
        spec.initialize(ObservationsNode)

        # Modify default node params
        spec.config.name = "env/observations"
        spec.config.rate = rate
        spec.config.process = process.ENVIRONMENT
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

    @register.inputs(actions_set=UInt64)
    @register.outputs(set=UInt64)
    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")


class ActionsNode(Node):
    @staticmethod
    @register.spec("Actions", Node)
    def spec(spec: NodeSpec, rate=1, log_level=WARN, color="yellow"):
        """ActionsNode spec"""
        # Initialize spec
        spec.initialize(ActionsNode)

        # Modify default node params
        spec.config.name = "env/actions"
        spec.config.rate = rate
        spec.config.process = process.ENVIRONMENT
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

    @register.inputs(observations_set=UInt64, step=UInt64)
    @register.outputs(set=UInt64)
    def callback(self, t_n: float, **kwargs: Optional[Msg]):
        raise NotImplementedError("This is a dummy class. Functionality is actually implemented in the Environment node.")


class RenderNode(Node):
    @staticmethod
    @register.spec("Render", Node)
    def spec(spec: NodeSpec, rate, display=True, log_level=WARN, color="grey"):
        """RenderNode spec"""
        # Initialize spec
        spec.initialize(RenderNode)

        # Modify default node params
        spec.config.name = "env/render"
        spec.config.rate = rate
        spec.config.process = process.ENVIRONMENT
        spec.config.color = color
        spec.config.log_level = log_level
        spec.config.inputs = ["image"]
        spec.config.outputs = ["done"]
        spec.config.states = []

        # Modify custom params
        spec.config.display = display

        # Pre-set window
        spec.inputs.image.window = 0

    def initialize(self, display):
        self.cv_bridge = cv_bridge.CvBridge()
        self.window = None
        self.display = display
        self.last_image = Image(data=[])
        self.render_toggle = False
        self.window_closed = True
        rospy.Subscriber("%s/%s/toggle" % (self.ns, self.name), Bool, self._set_render_toggle)
        rospy.Subscriber("%s/%s/get_last_image" % (self.ns, self.name), Bool, self._get_last_image)
        self.pub_set_last_image = rospy.Publisher(
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
        self.pub_set_last_image.publish(self.last_image)

    def reset(self):
        pass
        # self.last_image = Image()

    @register.inputs(image=Image)
    @register.outputs(done=UInt64)
    def callback(self, t_n: float, image: Optional[Msg] = None):
        if len(image.msgs) > 0:
            self.last_image = image.msgs[-1]
        empty = self.last_image.height == 0 or self.last_image.width == 0
        if not empty and self.display and self.render_toggle:
            try:
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.last_image, "bgr8")
                except ImportError as e:
                    rospy.logwarn_once("[%s] %s. Using numpy instead." % (self.ns_name, e))

                    if isinstance(self.last_image.data, bytes):
                        cv_image = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(
                            self.last_image.height, self.last_image.width, -1
                        )
                    else:
                        cv_image = np.array(self.last_image.data, dtype=np.uint8).reshape(
                            self.last_image.height, self.last_image.width, -1
                        )
                    if "rgb" in self.last_image.encoding:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            except cv_bridge.CvBridgeError as e:
                rospy.logwarn(e)
                return dict(done=UInt64())
            cv2.imshow("Render", cv_image)
            cv2.waitKey(1)
            self.window_closed = False
        elif not self.window_closed:
            cv2.destroyWindow("Render")
            self.window_closed = True

        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=UInt64())
        return output_msgs

    def shutdown(self):
        rospy.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        cv2.destroyAllWindows()

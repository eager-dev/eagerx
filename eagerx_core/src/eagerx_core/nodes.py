from typing import Optional

import numpy as np
import rospy
from std_msgs.msg import UInt64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import eagerx_core.registration as register
from eagerx_core.constants import process, DEBUG
from eagerx_core.entities import Node
from eagerx_core.utils.utils import initialize_converter, Msg
from eagerx_core.srv import ImageUInt8, ImageUInt8Response


class ObservationsNode(Node):
    @staticmethod
    @register.spec('Observations', Node)
    def spec(spec, rate=1, log_level=DEBUG, color='yellow'):
        """ObservationsNode spec"""
        # Initialize spec
        spec.initialize(ObservationsNode)

        # Modify default node params
        params = dict(name='env/observations',
                      rate=rate,
                      process=process.ENVIRONMENT,
                      color=color,
                      log_level=log_level,
                      inputs=['actions_set'],
                      outputs=['set'],
                      states=[])
        spec.set_parameters(params)

        # Pre-set address
        spec.set_component_parameter('inputs', 'actions_set', 'address', 'env/actions/outputs/set')

        # Pre-set window
        spec.set_component_parameter('inputs', 'actions_set', 'window', 0)

        # Set skip for first action_set (so that we do not block at t=0)
        spec.set_component_parameter('inputs', 'actions_set', 'skip', True)

    def initialize(self):
        # Define observation buffers
        self.observation_buffer = dict()
        for i in self.inputs:
            if i['name'] == 'actions_set':
                continue
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.observation_buffer[i['name']] = {'msgs': None, 'converter': converter, 'window': i['window']}

    def reset(self):
        # Set all messages to None
        for name, buffer in self.observation_buffer.items():
            buffer['msgs'] = None

    @register.inputs(actions_set=UInt64)
    @register.outputs(set=UInt64)
    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Set all observations to messages in inputs
        for name, buffer in self.observation_buffer.items():
            buffer['msgs'] = kwargs[name].msgs

        # Send output_msg
        output_msgs = dict(set=UInt64())
        return output_msgs


class ActionsNode(Node):
    @staticmethod
    @register.spec('Actions', Node)
    def spec(spec, rate=1, log_level=DEBUG, color='red'):
        """ActionsNode spec"""
        # Initialize spec
        spec.initialize(ActionsNode)

        # Modify default node params
        params = dict(name='env/actions',
                      rate=rate,
                      process=process.ENVIRONMENT,
                      color=color,
                      log_level=log_level,
                      inputs=['observations_set', 'step'],
                      outputs=['set'],
                      states=[])
        spec.set_parameters(params)

        # Pre-set addresses
        spec.set_component_parameter('inputs', 'observations_set', 'address', 'env/observations/outputs/set')
        spec.set_component_parameter('inputs', 'step', 'address', 'env/supervisor/outputs/step')

        # Pre-set window
        spec.set_component_parameter('inputs', 'observations_set', 'window', 0)
        spec.set_component_parameter('inputs', 'step', 'window', 0)

    def initialize(self):
        # Define action/observation buffers
        self.action_buffer = dict()
        for i in self.outputs:
            if i['name'] == 'set':
                continue
            if 'converter' in i and isinstance(i['converter'], dict):
                i['converter'] = initialize_converter(i['converter'])
                converter = i['converter']
            elif 'converter' in i and not isinstance(i['converter'], dict):
                converter = i['converter']
            else:
                converter = None
            self.action_buffer[i['name']] = {'msg': None, 'converter': converter}

    def reset(self):
        # Set all messages to None
        for name, buffer in self.action_buffer.items():
            buffer['msg'] = None
        # start_with an initial action message, so that the first observation can pass.
        return dict(set=UInt64())

    @register.inputs(observations_set=UInt64, step=UInt64)
    @register.outputs(set=UInt64)
    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Fill output_msg with buffered actions
        output_msgs = dict(set=UInt64())
        for name, buffer in self.action_buffer.items():
            output_msgs[name] = buffer['msg']
        return output_msgs


class RenderNode(Node):
    @staticmethod
    @register.spec('Render', Node)
    def spec(spec, rate, display=True, log_level=DEBUG, color='red'):
        """RenderNode spec"""
        # Initialize spec
        spec.initialize(RenderNode)

        # Modify default node params
        params = dict(name='env/render',
                      rate=rate,
                      process=process.NEW_PROCESS,
                      color=color,
                      log_level=log_level,
                      inputs=['image'],
                      outputs=['done'],
                      states=[])
        spec.set_parameters(params)

        # Modify custom params
        spec.set_parameter('display', display)

        # Pre-set window
        spec.set_component_parameter('inputs', 'image', 'window', 0)

    def initialize(self, display):
        self.cv_bridge = CvBridge()
        self.window = None
        self.display = display
        self.last_image = Image(data=[])
        self.render_toggle = False
        self.window_closed = True
        rospy.Service('%s/%s/get_last_image' % (self.ns, self.name), ImageUInt8, self._get_last_image)
        rospy.Subscriber('%s/%s/toggle' % (self.ns, self.name), Bool, self._set_render_toggle)

    def _set_render_toggle(self, msg):
        if msg.data:

            rospy.loginfo('START RENDERING!')
        else:
            rospy.loginfo('STOP RENDERING!')
        self.render_toggle = msg.data

    def _get_last_image(self, req):
        return ImageUInt8Response(image=self.last_image)

    def reset(self):
        self.last_image = Image()

    @register.inputs(image=Image)
    @register.outputs(done=UInt64)
    def callback(self, node_tick: int, t_n: float, image: Optional[Msg] = None):
        if len(image.msgs) > 0:
            self.last_image = image.msgs[-1]
        empty = (self.last_image.height == 0 or self.last_image.width == 0)
        if not empty and self.display and self.render_toggle:
            try:
                try:
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
                except ImportError as e:
                    rospy.logwarn_once('[%s] %s. Using numpy instead.' % (self.ns_name, e))

                    if isinstance(self.last_image.data, bytes):
                        cv_image = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(self.last_image.height, self.last_image.width, -1)
                    else:
                        cv_image = np.array(self.last_image.data, dtype=np.uint8).reshape(self.last_image.height, self.last_image.width, -1)
                    if 'rgb' in self.last_image.encoding:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            except CvBridgeError as e:
                rospy.logwarn(e)
                return dict(done=UInt64())
            cv2.imshow('Render', cv_image)
            cv2.waitKey(1)
            self.window_closed = False
        elif not self.window_closed:
            cv2.destroyWindow('Render')
            self.window_closed = True

        # Fill output_msg with 'done' output --> signals that we are done rendering
        output_msgs = dict(done=UInt64())
        return output_msgs

from typing import Optional

import numpy as np
import rospy
from std_msgs.msg import UInt64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from eagerx_core.entities import Node
from eagerx_core.utils.utils import initialize_converter, Msg
from eagerx_core.srv import ImageUInt8, ImageUInt8Response


class ObservationsNode(Node):
    msg_types = {'inputs': {'actions_set': UInt64},
                 'outputs': {'set': UInt64}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
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

    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Set all observations to messages in inputs
        for name, buffer in self.observation_buffer.items():
            buffer['msgs'] = kwargs[name].msgs

        # Send output_msg
        output_msgs = dict(set=UInt64())
        return output_msgs


class ActionsNode(Node):
    msg_types = {'inputs': {'observations_set': UInt64,
                            'step': UInt64},
                 'outputs': {'set': UInt64}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
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

    def callback(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        # Fill output_msg with buffered actions
        output_msgs = dict(set=UInt64())
        for name, buffer in self.action_buffer.items():
            output_msgs[name] = buffer['msg']
        return output_msgs


class RenderNode(Node):

    msg_types = {'inputs': {'image': Image},
                 'outputs': {'done': UInt64}}

    def __init__(self, display, **kwargs):
        super().__init__(**kwargs)
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

    def callback(self, node_tick: int, t_n: float, image: Optional[Msg] = None):
        if len(image.msgs) > 0:
            self.last_image = image.msgs[-1]
        # todo: REMOVEEEE!!!!
        if self.display and self.render_toggle:
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

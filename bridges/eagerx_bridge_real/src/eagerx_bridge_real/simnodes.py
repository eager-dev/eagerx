from typing import Optional
import numpy as np
import cv2
import rospy

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# IMPORT EAGERX
from eagerx_core.utils.utils import return_typehint, Msg
from eagerx_core.nodes import SimNode
from eagerx_core.constants import process

from dcsc_fpga.srv import MopsWrite, MopsWriteRequest, MopsReadRequest, MopsRead

class PendulumOutput(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'observation': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.service = rospy.ServiceProxy('/mops/read', MopsRead)
        self.service.wait_for_service()

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32MultiArray):
        response = self.service.call(MopsReadRequest())
        data = [np.pi - response.sensors.position0, response.sensors.speed]
        return dict(observation=Float32MultiArray(data=data))

class PendulumRender(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'image': Image}}

    def __init__(self, shape, **kwargs):
        super().__init__(**kwargs)
        self.cv_bridge = CvBridge()
        self.shape = tuple(shape)
        self.always_render = self.object_params['always_render']
        self.render_toggle = False
        self.render_toggle_pub = rospy.Subscriber('%s/env/render/toggle' % self.ns, Bool, self._set_render_toggle)

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo('[%s] START RENDERING!' % self.name)
        else:
            rospy.loginfo('[%s] STOPPED RENDERING!' % self.name)
        self.render_toggle = msg.data

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Image):
        if self.always_render or self.render_toggle:
            width, height = self.shape
            l = width // 3
            img = np.zeros((height, width, 3), np.uint8)
            img = cv2.circle(img, (width // 2, height // 2), height // 2, (255, 0, 0), -1)
            img = cv2.circle(img, (width // 2, height // 2), height // 8, (192, 192, 192), -1)
            if self.simulator[self.obj_name]['state'] is not None:
                theta, _ = self.simulator[self.obj_name]['state']
                img = cv2.circle(img, (width // 2 + int(l * np.sin(theta)), height // 2 - int(l * np.cos(theta))),
                                 height // 6, (192, 192, 192), -1)
            msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
        else:
            msg = Image()
        return dict(image=msg)


class PendulumInput(SimNode):
    msg_types = {'inputs': {'tick': UInt64,
                            'action': Float32MultiArray},
                 'outputs': {'action_applied': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.service = rospy.ServiceProxy('/mops/write', MopsWrite)
        self.service.wait_for_service()

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None,
                 action: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):

        input = np.squeeze(action.msgs[-1].data)
        if input is not None:
            req = MopsWriteRequest()
            req.actuators.digital_outputs = 1
            req.actuators.voltage0 = input
            req.actuators.voltage1 = 0.0
            req.actuators.timeout = 0.5
            self.service(req)
        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])

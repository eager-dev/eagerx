from typing import Optional
import numpy as np
import rospy
import cv2

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# IMPORT EAGERX
from eagerx_core.utils.utils import return_typehint, Msg
from eagerx_core.nodes import SimNode

from dcsc_fpga.srv import MopsWrite, MopsWriteRequest, MopsReadRequest, MopsRead

class MopsOutput(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'mops_output': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.service = rospy.ServiceProxy('/mops/read', MopsRead)
        self.service.wait_for_service()

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32MultiArray):
        response = self.service.call(MopsReadRequest())
        data = [np.pi - response.sensors.position0, response.sensors.speed]
        return dict(mops_output=Float32MultiArray(data=data))


class MopsInput(SimNode):
    msg_types = {'inputs': {'tick': UInt64,
                            'mops_input': Float32MultiArray},
                 'outputs': {'action_applied': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.service = rospy.ServiceProxy('/mops/write', MopsWrite)
        self.service.wait_for_service()

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None,
                 mops_input: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        if len(mops_input.msgs) > 0:
            input = np.squeeze(mops_input.msgs[-1].data)
            if input is not None:
                req = MopsWriteRequest()
                req.actuators.digital_outputs = 1
                req.actuators.voltage0 = input
                req.actuators.voltage1 = 0.0
                req.actuators.timeout = 0.5
                self.service(req)
            # Send action that has been applied.
        else:
            input = 0
        return dict(action_applied=Float32MultiArray(data=[input]))


class MopsRender(SimNode):
    msg_types = {'inputs': {'tick': UInt64,
                            'theta': Float32MultiArray},
                 'outputs': {'image': Image}}

    def __init__(self, shape, **kwargs):
        super().__init__(**kwargs)
        self.cv_bridge = CvBridge()
        self.shape = tuple(shape)
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

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None, theta: Optional[Float32MultiArray] = None) -> return_typehint(Image):
        data = theta.msgs[-1].data
        if self.render_toggle:
            width, height = self.shape
            l = width // 3
            img = np.zeros((height, width, 3), np.uint8)
            img = cv2.circle(img, (width // 2, height // 2), height // 2, (255, 0, 0), -1)
            img = cv2.circle(img, (width // 2, height // 2), height // 8, (192, 192, 192), -1)
            if len(np.squeeze(data)) == 3:
                sin_theta, cos_theta, _ = np.squeeze(data)
                img = cv2.circle(img, (width // 2 + int(l * sin_theta), height // 2 - int(l * cos_theta)), height // 6,
                                 (192, 192, 192), -1)
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
            except ImportError as e:
                rospy.logwarn_once('[%s] %s. Using numpy instead.' % (self.ns_name, e))
                data = img.tobytes('C')
                msg = Image(data=data, height=height, width=width, encoding='bgr8')
        else:
            msg = Image()
        return dict(image=msg)

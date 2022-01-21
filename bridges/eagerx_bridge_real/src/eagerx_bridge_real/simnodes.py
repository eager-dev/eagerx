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

    def __init__(self, shape, camera_idx, **kwargs):
        super().__init__(**kwargs)
        self.cv_bridge = CvBridge()
        self.cam = None
        self.width, self.height = shape
        self.camera_idx = camera_idx
        self.render_toggle = False
        self.render_toggle_pub = rospy.Subscriber('%s/env/render/toggle' % self.ns, Bool, self._set_render_toggle)

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo('[%s] START RENDERING!' % self.name)
            self.cam = cv2.VideoCapture(self.camera_idx)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        else:
            rospy.loginfo('[%s] STOPPED RENDERING!' % self.name)
            if self.cam is not None:
                self.cam.release()
                self.cam = None
        self.render_toggle = msg.data

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Image):
        if self.render_toggle:
            ret, img = self.cam.read()
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
            except ImportError as e:
                rospy.logwarn_once('[%s] %s. Using numpy instead.' % (self.ns_name, e))
                data = img.tobytes('C')
                msg = Image(data=data, height=self.height, width=self.width, encoding='bgr8')
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
        if len(action.msgs) > 0:
            input = np.squeeze(action.msgs[-1].data)
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

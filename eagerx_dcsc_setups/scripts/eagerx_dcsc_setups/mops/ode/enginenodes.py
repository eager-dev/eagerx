from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt64, Float32MultiArray

from eagerx.core import register as register
from eagerx.core.constants import process
from eagerx.core.entities import EngineNode
from eagerx.utils.utils import Msg, return_typehint


class MopsImage(EngineNode):
    @staticmethod
    @register.spec('MopsImage', EngineNode)
    def spec(spec, name: str, rate: float, process: Optional[int] = process.NEW_PROCESS,
             color: Optional[str] = 'cyan', shape=[480, 480]):
        """MopsImage spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(MopsImage)

        # Modify default node params
        params = dict(name=name, rate=rate, process=process, color=color, inputs=['tick', 'theta'], outputs=['image'])
        spec.set_parameters(params)

        # Modify custom node params
        spec.set_parameter('shape', shape)

        # Set component parameter
        spec.set_component_parameter('inputs', 'theta', 'window', 1)

    def initialize(self, shape):
        self.cv_bridge = CvBridge()
        self.shape = tuple(shape)
        self.render_toggle = False
        self.render_toggle_pub = rospy.Subscriber('%s/env/render/toggle' % self.ns, Bool, self._set_render_toggle)

    @register.states()
    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    @register.inputs(tick=UInt64, theta=Float32MultiArray)
    @register.outputs(image=Image)
    def callback(self, t_n: float, tick: Optional[Msg] = None, theta: Optional[Float32MultiArray] = None) -> return_typehint(Image):
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

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo('[%s] START RENDERING!' % self.name)
        else:
            rospy.loginfo('[%s] STOPPED RENDERING!' % self.name)
        self.render_toggle = msg.data
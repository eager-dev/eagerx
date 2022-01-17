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


class OdeOutput(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'observation': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.obj_name = self.object_params['name']

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), \
            'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        data = self.simulator[self.obj_name]['state']
        return dict(observation=Float32MultiArray(data=data))

class PendulumRender(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'image': Image}}

    def __init__(self, shape, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.cv_bridge = CvBridge()
        self.shape = tuple(shape)
        self.always_render = self.object_params['always_render']
        self.render_toggle = False
        self.obj_name = self.object_params['name']
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
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
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


class OdeInput(SimNode):
    msg_types = {'inputs': {'tick': UInt64,
                            'action': Float32MultiArray},
                 'outputs': {'action_applied': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, \
            'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.obj_name = self.object_params['name']

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None,
                 action: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), \
            'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]

        # Set action in simulator for next step.
        self.simulator[self.obj_name]['input'] = np.squeeze(action.msgs[-1].data)

        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])

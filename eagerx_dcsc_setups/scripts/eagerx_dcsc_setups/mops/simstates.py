import rospy
from random import random
from eagerx_core.simstates import SimStateBase
from dcsc_fpga.srv import MopsWrite, MopsWriteRequest

class RandomActionAndSleep(SimStateBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.service = rospy.ServiceProxy('/mops/write', MopsWrite)
        self.service.wait_for_service()

    def reset(self, state, done):
        action = -3. + random() * 6.
        req = MopsWriteRequest()
        req.actuators.digital_outputs = 1
        req.actuators.voltage0 = action
        req.actuators.voltage1 = 0.0
        req.actuators.timeout = 0.5
        self.service(req)
        rospy.sleep(1.)
        return None

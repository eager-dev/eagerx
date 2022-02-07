import rospy
from random import random
from eagerx.core.entities import EngineState
import eagerx.core.register as register
from dcsc_fpga.srv import MopsWrite, MopsWriteRequest


class RandomActionAndSleep(EngineState):
    @staticmethod
    @register.spec('RandomActionAndSleep', EngineState)
    def spec(spec, sleep_time: float = 1., repeat: int = 1):
        spec.set_parameter('sleep_time', sleep_time)
        spec.set_parameter('repeat', repeat)

    def initialize(self, sleep_time: float, repeat: int):
        self.sleep_time = sleep_time
        self.repeat = repeat
        self.service = rospy.ServiceProxy('/mops/write', MopsWrite)
        self.service.wait_for_service()

    def reset(self, state, done):
        for i in range(self.repeat):
            action = -3. + random() * 6.
            req = MopsWriteRequest()
            req.actuators.digital_outputs = 1
            req.actuators.voltage0 = action
            req.actuators.voltage1 = 0.0
            req.actuators.timeout = 0.5
            self.service(req)
            rospy.sleep(self.sleep_time)

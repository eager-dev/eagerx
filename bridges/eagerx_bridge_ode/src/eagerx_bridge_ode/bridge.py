# OTHER
from typing import Optional, Dict, Union, List

import numpy as np
from math import fmod
from scipy.integrate import odeint

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx_core.utils.utils import Msg, get_attribute_from_module
from eagerx_core.bridge import BridgeBase


class OdeBridge(BridgeBase):
    msg_types = {'outputs': {'tick': UInt64}}

    def __init__(self, rtol, atol, hmax, hmin, mxstep, model_noise, **kwargs):
        # Initialize any simulator here, that is passed as reference to each simnode
        self.odeint_args = dict(rtol=rtol, atol=atol, hmax=hmax, hmin=hmin, mxstep=mxstep)
        self.model_noise = model_noise
        simulator = dict()
        super().__init__(simulator=simulator, **kwargs)

    def add_object_to_simulator(self, object_params, node_params, state_params) -> Dict:
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo('Adding object "%s" of type "%s.yaml" from package "%s" to the simulator.' % (
        object_params['name'], object_params['config_name'], object_params['package_name']))

        # Extract relevant object_params
        obj_name = object_params['name']
        ode = get_attribute_from_module(object_params['bridge']['ode'])
        Dfun = get_attribute_from_module(object_params['bridge']['Dfun']) if 'Dfun' in object_params else None

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(ode=ode, Dfun=Dfun, state=None, input=None)
        return object_params

    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    def reset(self, **kwargs: Optional[Msg]):
        pass

    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        for obj_name, sim in self.simulator.items():
            input = sim['input']
            ode = sim['ode']
            Dfun = sim['Dfun']
            x = sim['state']
            sim['state'] = odeint(ode, x, [0, 1./self.rate], args=(input,), Dfun=Dfun, **self.odeint_args)[-1]

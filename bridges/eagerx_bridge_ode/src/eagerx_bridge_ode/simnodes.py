from typing import Optional
import numpy as np

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray

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


class ActionApplied(SimNode):
    msg_types = {'inputs': {'tick': UInt64, 'action_applied': Float32MultiArray},
                 'outputs': {'action_applied': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.obj_name = self.object_params['name']

    def reset(self):
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None,
                 action_applied: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        if len(action_applied.msgs) > 0:
            data = action_applied.msgs[-1].data
        else:
            data = [0]
        return dict(action_applied=Float32MultiArray(data=data))


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

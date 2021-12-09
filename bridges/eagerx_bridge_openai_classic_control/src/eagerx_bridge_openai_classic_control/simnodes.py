from typing import Dict, List, Union
import numpy as np

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray, MultiArrayDimension

# IMPORT EAGERX
from eagerx_core.basenode import SimNode
from eagerx_core.constants import process


class TorqueControl(SimNode):
    def __init__(self, object_params, **kwargs):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.obj_name = object_params['name']
        super().__init__(object_params=object_params, **kwargs)

    def reset(self):
        # This controller is stateless (in contrast to e.g. a PID controller).
        self.simulator[self.obj_name]['next_action'] = None

    def callback(self, node_tick: int, t_n: float,
                 tick: Dict[str, Union[List[UInt64], float, int]] = None,
                 torque: Dict[str, Union[List[Float32MultiArray], float, int]] = None) -> Dict[str, Float32MultiArray]:
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]

        # Set action in simulator for next step.
        self.simulator[self.obj_name]['next_action'] = torque['msg'][-1].data

        # Send action that has been applied.
        return dict(torque_applied=torque['msg'][-1])


class JointSensor(SimNode):
    def __init__(self, mode, object_params, **kwargs):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.mode = mode
        self.id = object_params['bridge']['id']
        self.obj_name = object_params['name']
        super().__init__(object_params=object_params, **kwargs)

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def callback(self, node_tick: int, t_n: float,
                 tick: Dict[str, Union[List[UInt64], float, int]] = None) -> Dict[str, Float32MultiArray]:
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        if self.id == 'Pendulum-v0':
            if len(self.mode) == 2:
                obs = self.simulator[self.obj_name]['last_obs']
            else:
                raise ValueError('Other modes (only pos/vel) not yet implemented.')
        else:
            raise ValueError('Env "%s" not yet supported.' % self.id)
        return dict(joint_obs=Float32MultiArray(data=obs))

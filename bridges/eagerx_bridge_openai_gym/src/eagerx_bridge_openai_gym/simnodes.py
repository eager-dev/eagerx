from typing import Dict, List, Union, Optional
import skimage.transform
import numpy as np
import gym

# IMPORT ROS
import rospy
from std_msgs.msg import UInt64, Float32MultiArray, Bool, Float32
from sensor_msgs.msg import Image

# IMPORT EAGERX
from eagerx_core.utils.utils import return_typehint, Msg
from eagerx_core.nodes import SimNode
from eagerx_core.constants import process


class ObservationSensor(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'observation': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.id = self.object_params['gym_id']
        self.obj_name = self.object_params['name']
        self.last_obs = None

    def reset(self):
        self.last_obs = None

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        obs = self.simulator[self.obj_name]['buffer_obs']
        self.simulator[self.obj_name]['buffer_obs'] = []
        if len(obs) == 0:
            obs = self.last_obs
        else:
            obs = obs[-1]
            self.last_obs = obs
        return dict(observation=Float32MultiArray(data=obs))


class RewardSensor(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'reward': Float32}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.id = self.object_params['gym_id']
        self.obj_name = self.object_params['name']
        self.last_reward = None

    def reset(self):
        self.last_reward = 0.

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        reward = self.simulator[self.obj_name]['buffer_reward']
        self.simulator[self.obj_name]['buffer_reward'] = []
        if len(reward) == 0:
            reward = self.last_reward
        else:
            self.last_reward = reward[-1]
            reward = sum(reward)
        return dict(reward=Float32(data=reward))


class DoneSensor(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'done': Bool}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.id = self.object_params['gym_id']
        self.obj_name = self.object_params['name']
        self.last_done = None

    def reset(self):
        self.last_done = False

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Bool):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        done = self.simulator[self.obj_name]['buffer_done']
        self.simulator[self.obj_name]['buffer_done'] = []
        if len(done) == 0:
            done = self.last_done
        else:
            done = any(done) or self.last_done
            self.last_done = done
        return dict(done=Bool(data=done))


class ActionActuator(SimNode):
    msg_types = {'inputs': {'tick': UInt64,
                            'action': Float32MultiArray},
                 'outputs': {'action_applied': Float32MultiArray}}

    def __init__(self, zero_action, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.obj_name = self.object_params['name']
        self.simulator[self.obj_name]['env']: gym.Env
        self.is_discrete = True if isinstance(self.simulator[self.obj_name]['env'].action_space, gym.spaces.Discrete) else False
        if zero_action == 'not_defined':
            self.zero_action = self.simulator[self.obj_name]['env'].action_space.sample()
        else:
            if isinstance(zero_action, list):
                dtype = self.simulator[self.obj_name]['env'].action_space.dtype
                self.zero_action = np.array(zero_action, dtype=dtype)
            else:
                self.zero_action = zero_action
            assert self.simulator[self.obj_name]['env'].action_space.contains(self.zero_action), 'The zero action provided for "%s" is not contained in the action space of this environment.' % self.obj_name

    def reset(self):
        # This controller is stateless (in contrast to e.g. a PID controller).
        self.simulator[self.obj_name]['next_action'] = self.zero_action

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None, action: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]

        # Set action in simulator for next step.
        if len(action.msgs) > 0:
            self.simulator[self.obj_name]['next_action'] = action.msgs[-1].data[0] if self.is_discrete else action.msgs[-1].data
        else:
            self.simulator[self.obj_name]['next_action'] = self.zero_action

        # Prepare output message
        action_applied = self.simulator[self.obj_name]['next_action']

        # Send action that has been applied.
        return dict(action_applied=Float32MultiArray(data=action_applied))


class GymRenderer(SimNode):
    msg_types = {'inputs': {'tick': UInt64},
                 'outputs': {'image': Image}}

    def __init__(self, shape, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.shape = tuple(shape)
        self.always_render = self.object_params['always_render']
        self.render_toggle = False
        self.id = self.object_params['gym_id']
        self.obj_name = self.object_params['name']
        self.render_toggle_pub = rospy.Subscriber('%s/env/render/toggle' % self.ns, Bool, self._set_render_toggle)

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo('[%s] START RENDERING!' % self.name)
        else:
            self.simulator[self.obj_name]['env'].close()
            rospy.loginfo('[%s] STOPPED RENDERING!' % self.name)
        self.render_toggle = msg.data

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def _show_ros_image(self, msg):
        import matplotlib.pyplot as plt
        rgb_back = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        plt.imshow(rgb_back)
        plt.show()

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Image):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        if self.always_render or self.render_toggle:
            rgb = self.simulator[self.obj_name]['env'].render(mode="rgb_array")

            # Resize image if not matching desired self.shape (defined in .yaml)
            if rgb.shape[:2] != tuple(self.shape):
                kwargs = dict(output_shape=self.shape, mode='edge', order=1, preserve_range=True)
                rgb = skimage.transform.resize(rgb, **kwargs).astype(rgb.dtype)

            # Prepare ROS msg
            height = rgb.shape[0]
            width = rgb.shape[1]
            msg = Image(data=rgb.reshape(-1).tolist(), height=height, width=width, encoding='rgb8')
            # self._show_ros_image(msg)
        else:
            msg = Image()
        return dict(image=msg)
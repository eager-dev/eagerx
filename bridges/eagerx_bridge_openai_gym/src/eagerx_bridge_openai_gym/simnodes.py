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
from eagerx_core.basenode import SimNode
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

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        obs = self.simulator[self.obj_name]['last_obs']
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

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Float32):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        reward = self.simulator[self.obj_name]['last_reward']
        if reward is None:
            reward = 0.
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

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None) -> return_typehint(Bool):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]
        done = self.simulator[self.obj_name]['last_done']
        if done is None:
            done = False
        return dict(done=Bool(data=done))


class ActionActuator(SimNode):
    msg_types = {'inputs': {'tick': UInt64,
                            'action': Float32MultiArray},
                 'outputs': {'action_applied': Float32MultiArray}}

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        assert kwargs['process'] == process.BRIDGE, 'Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process'
        self.obj_name = self.object_params['name']
        self.is_discrete = True if isinstance(self.simulator[self.obj_name]['env'].action_space, gym.spaces.Discrete) else False

    def reset(self):
        # This controller is stateless (in contrast to e.g. a PID controller).
        self.simulator[self.obj_name]['next_action'] = None

    def callback(self, node_tick: int, t_n: float, tick: Optional[Msg] = None, action: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        assert isinstance(self.simulator[self.obj_name], dict), 'Simulator object "%s" is not compatible with this simulation node.' % self.simulator[self.obj_name]

        # Set action in simulator for next step.
        self.simulator[self.obj_name]['next_action'] = action.msgs[-1].data[0] if self.is_discrete else action.msgs[-1].data

        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])


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
            msg = Image(data=rgb.reshape(-1).tolist(), height=height, width=width)
            # self._show_ros_image(msg)
        else:
            msg = Image()
        return dict(image=msg)
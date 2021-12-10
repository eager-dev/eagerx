from typing import Dict, List, Union
import skimage.transform
import numpy as np
import h5py

# IMPORT ROS
import rospy
from std_msgs.msg import UInt64, Float32MultiArray, Bool
from sensor_msgs.msg import Image

# IMPORT EAGERX
from eagerx_core.basenode import Node
from eagerx_core.utils.utils import substitute_xml_args


class Classifier(Node):
    def __init__(self, data_path, **kwargs):
        self._train_model(data_path)
        super().__init__(**kwargs)

    def _train_model(self, data_path):
        # todo: train an inference model
        data_path = substitute_xml_args(data_path)
        # LOAD DATASET
        with h5py.File(data_path, 'r') as hf:
            observation = hf['observation.h5'][:]
            state = hf['state.h5'][:]
        print('Loaded observation data: %s' % str(observation.shape))
        print('Loaded state data: %s' % str(state.shape))

        # DATASET PARAMETERS
        num_examples = 12000
        observation = observation[:num_examples]
        state = state[:num_examples]

        # DATA PRE-PROCESSING
        # Scale pixel values to a range of 0 to 1 before feeding them to the neural network model.
        observation = observation.astype(np.float32) / 255.

        # CREATE TEST DATASET
        test_split = 0.2
        if 0 < test_split < 1:
            split_at = int(len(observation) * (1 - test_split))
        else:
            raise ValueError('Must hold-out data as a test dataset. Set parameter 0 < test_split < 1.')
        test_obs = observation[split_at:, :, :, :]
        test_theta = state[split_at:, 0]
        test_trig = np.hstack([np.sin(test_theta)[:, None], np.cos(test_theta)[:, None]])

        # CREATE TRAINING DATASET
        train_obs = observation[:split_at, :, :, :]
        train_theta = state[:split_at, 0]
        train_trig = np.hstack([np.sin(train_theta)[:, None], np.cos(train_theta)[:, None]])

    def reset(self):
        # This sensor is stateless (in contrast to e.g. a PID controller).
        pass

    def _show_ros_image(self, msg):
        import matplotlib.pyplot as plt
        rgb_back = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        plt.imshow(rgb_back)
        plt.show()

    def callback(self, node_tick: int, t_n: float,
                 tick: Dict[str, Union[List[UInt64], float, int]] = None,
                 pos_vel: Dict[str, Union[List[Float32MultiArray], float, int]] = None,
                 image: Dict[str, Union[List[Image], float, int]] = None) -> Dict[str, Float32MultiArray]:
        # todo: infer joint_obs from image --> probably theta, while we must output sin(theta), cos(theta)
        # todo: infer joint_vel via finite differencing.

        # Prepare ROS msg
        # return dict(estimated_pos_vel=Float32MultiArray(data=INSERT HERE))
        return dict(estimated_pos_vel=pos_vel['msg'][-1])

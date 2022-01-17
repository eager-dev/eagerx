#!/usr/bin/env python3

# ROS packages required
import os
import rospy
import numpy as np
from functools import partial
from eagerx_core.core import RxObject, RxGraph, RxBridge, EAGERxEnv
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten

import stable_baselines3 as sb

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore
    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define rate (depends on rate of ode)
    rate = 30.
    # Decompose angle in
    decompose_angle = True
    real = False

    graph = RxGraph.create()
    pendulum = RxObject.create('pendulum', 'eagerx_bridge_ode', 'pendulum')
    graph.add(pendulum)
    graph.add_component('pendulum', 'sensors', 'image')
    graph.connect(action='action', target=('pendulum', 'actuators', 'action'))
    if decompose_angle:
        graph.set_parameter('converter', {'converter_type': 'eagerx_bridge_ode.converters/AngleDecomposition'},
                            'pendulum', 'sensors', 'observation')
        graph.connect(source=('pendulum', 'sensors', 'observation'), observation='observation',
                      converter={'converter_type': 'eagerx_bridge_ode.converters/Space_RosFloat32MultiArray',
                                 'low': [-1, -1, -9], 'high': [1, 1, 9]})
    else:
        graph.set_parameter('converter', {'converter_type': 'eagerx_bridge_ode.converters/AngleNormalization'},
                            'pendulum', 'sensors', 'observation')
        graph.connect(source=('pendulum', 'sensors', 'observation'), observation='observation')
    graph.render(source=('pendulum', 'sensors', 'image'), rate=20, display=True)

    # Define bridge
    if real:
        bridge = RxBridge.create('eagerx_bridge_real', 'bridge', rate=rate, is_reactive=False,
                                 process=process.NEW_PROCESS)
    else:
        bridge = RxBridge.create('eagerx_bridge_ode', 'bridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.NEW_PROCESS)

    def angle_normalize(x):
        return ((x + np.pi) % (2 * np.pi)) - np.pi

    def reward_fn(prev_obs, obs, action, steps, decompose_angle=True):
        if decompose_angle:
            sin_th, cos_th, thdot = np.squeeze(obs['observation'])
            th = np.arctan2(sin_th, cos_th)
        else:
            th, thdot = np.squeeze(obs['observation'])
            th = angle_normalize(th)
        cost = th ** 2 + 0.1 * (thdot / (1 + 10 * abs(th))) ** 2
        return - cost


    # Initialize Environment
    env = EAGERxEnv(
        name='rx', rate=rate, graph=graph, bridge=bridge,
        is_done_fn=lambda obs, action, steps: steps > 300 or np.all(np.abs(obs['observation']) < 0.1),
        reward_fn=partial(reward_fn, decompose_angle=decompose_angle),
    )
    env = Flatten(env)

    # Use stable-baselines
    parent_folder = os.path.dirname(os.path.realpath(__file__))
    save_path = '{}/pendulum'.format(parent_folder)
    # model = sb.SAC.load(save_path, env=env)
    model = sb.SAC('MlpPolicy', env, verbose=1, tensorboard_log='{}/log'.format(parent_folder))
    model.learn(total_timesteps=int(1800*rate))
    model.save(save_path)
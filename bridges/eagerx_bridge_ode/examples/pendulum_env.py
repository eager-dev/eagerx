#!/usr/bin/env python3

# ROS packages required
import os
import rospy
import numpy as np
from eagerx_core.core import RxObject, RxNode, RxGraph, RxBridge, EAGERxEnv
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten

import stable_baselines3 as sb

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore
    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define rate (depends on rate of ode)
    rate = 30.
    real = True
    render = True

    graph = RxGraph.create()
    pendulum = RxObject.create('pendulum', 'eagerx_bridge_ode', 'pendulum')
    butterworth_filter = RxNode.create('butterworth_filter', 'eagerx_bridge_real', 'butterworth_filter', N=2, Wn=14, rate=rate)

    graph.add(pendulum)
    graph.add(butterworth_filter)

    graph.set_parameter(
        'space_converter',
        {'converter_type': 'eagerx_bridge_ode.converters/Space_RosFloat32MultiArray', 'low': [-3], 'high': [3]},
        'butterworth_filter', 'inputs', 'signal',
    )
    graph.connect(action='action', target=('butterworth_filter', 'inputs', 'signal'))
    graph.connect(source=('butterworth_filter', 'outputs', 'filtered'), target=('pendulum', 'actuators', 'action'))
    graph.connect(source=('pendulum', 'sensors', 'observation'), observation='observation', window=1)
    graph.connect(source=('pendulum', 'sensors', 'action_applied'), observation='action_applied', window=1)

    if real:
        graph.remove_component('pendulum', 'states', 'model_state')
    elif render:
        graph.add_component('pendulum', 'sensors', 'image')
        graph.render(source=('pendulum', 'sensors', 'image'), rate=20, display=True)

    graph.remove('butterworth_filter', remove=True)
    graph.connect(action='action', target=('pendulum', 'actuators', 'action'))
    graph.remove_component('pendulum', 'sensors', 'action_applied', remove=True)

    # graph.gui()

    # Define bridge
    if real:
        bridge = RxBridge.create('eagerx_bridge_real', 'bridge', rate=rate, is_reactive=False, process=process.NEW_PROCESS)
    else:
        bridge = RxBridge.create('eagerx_bridge_ode', 'bridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.NEW_PROCESS)

    def reward_fn(prev_obs, obs, action, steps):
        data = np.squeeze(obs['observation'])
        if len(data) == 3:
            sin_th, cos_th, thdot = np.squeeze(obs['observation'])
        else:
            sin_th, cos_th, thdot = 0, -1, 0
        th = np.arctan2(sin_th, cos_th)
        cost = th ** 2 + 0.1 * (thdot / (1 + 10 * abs(th))) ** 2
        return - cost

    def is_done_fn(obs, action, steps):
        done = False
        if steps > 500:
            done = True
        data = np.squeeze(obs['observation'])
        if len(data) == 3:
            sin_th, cos_th, thdot = np.squeeze(obs['observation'])
        else:
            sin_th, cos_th, thdot = 0, -1, 0
        th = np.arctan2(sin_th, cos_th)
        if np.all(np.abs([th, thdot]) < 0.01):
            done = True
        return done


    # Initialize Environment
    env = EAGERxEnv(name='rx', rate=rate, graph=graph, bridge=bridge, is_done_fn=is_done_fn, reward_fn=reward_fn)
    env = Flatten(env)
    env.render('human')

    # Use stable-baselines
    parent_folder = os.path.dirname(os.path.realpath(__file__))
    save_path = '{}/pendulum_ppo'.format(parent_folder)
    # model = sb.SAC.load(save_path, env=env)
    model = sb.PPO("MlpPolicy", env, verbose=1)
    # model = sb.SAC("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=int(360*rate))
    # model.save(save_path)

    rospy.loginfo('Start Evaluation!')
    obs = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

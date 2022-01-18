#!/usr/bin/env python3

# ROS packages required
import os
import rospy
import numpy as np
from gym import spaces
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
    real = True
    render = True

    graph = RxGraph.create()
    pendulum = RxObject.create('pendulum', 'eagerx_bridge_ode', 'pendulum')
    graph.add(pendulum)
    graph.connect(action='action', target=('pendulum', 'actuators', 'action'))
    graph.set_parameter('converter', {'converter_type': 'eagerx_bridge_ode.converters/AngleDecomposition'},
                        'pendulum', 'sensors', 'observation')
    graph.connect(source=('pendulum', 'sensors', 'observation'), observation='observation',
                  converter={'converter_type': 'eagerx_bridge_ode.converters/Space_RosFloat32MultiArray',
                             'low': [-1, -1, -9], 'high': [1, 1, 9]})

    if real:
        graph.remove_component('pendulum', 'states', 'model_state')
    elif render:
        graph.add_component('pendulum', 'sensors', 'image')
        graph.render(source=('pendulum', 'sensors', 'image'), rate=20, display=True)

    # Define bridge
    if real:
        bridge = RxBridge.create('eagerx_bridge_real', 'bridge', rate=rate, is_reactive=False, process=process.NEW_PROCESS)
    else:
        bridge = RxBridge.create('eagerx_bridge_ode', 'bridge', rate=rate, is_reactive=True, process=process.NEW_PROCESS)

    class NormalizeActions(object):
        """
        if and(low, high) not np.inf: scales action from [-1, 1] back to the unnormalized action
        if or(low,high) np.inf: no normalization of the actions, and true action must be used"""

        def __init__(self, env):
            self._env = env
            low, high = env.action_space.low, env.action_space.high
            self._enabled = np.logical_and(np.isfinite(low), np.isfinite(high))
            self._low = np.where(self._enabled, low, -np.ones_like(low))
            self._high = np.where(self._enabled, high, np.ones_like(low))

        def __getattr__(self, name):
            return getattr(self._env, name)

        @property
        def action_space(self):
            space = self._env.action_space
            low = np.where(self._enabled, -np.ones_like(space.low), space.low)
            high = np.where(self._enabled, np.ones_like(space.high), space.high)
            return spaces.Box(low, high, dtype=space.dtype)

        def step(self, action):
            # de-normalize action
            action = self.denormalize_action(action)

            # apply action
            obs, reward, done, info = self._env.step(action)
            return obs, reward, done, info

        def denormalize_action(self, action):
            return (action + 1) * (self._high - self._low) / 2 + self._low


    def reward_fn(prev_obs, obs, action, steps):
        sin_th, cos_th, thdot = np.squeeze(obs['observation'])
        th = np.arctan2(sin_th, cos_th)
        if np.all(np.abs([th, thdot]) < 0.01):
            return 100
        u = np.squeeze(action['action'])
        cost = th ** 2 + 0.1 * (thdot / (1 + 10 * abs(th))) ** 2 + 0.001 * (u ** 2)
        return - cost

    def is_done_fn(obs, action, steps):
        done = False
        if steps > 500:
            done = True
        sin_th, cos_th, thdot = np.squeeze(obs['observation'])
        th = np.arctan2(sin_th, cos_th)
        if np.all(np.abs([th, thdot]) < 0.01):
            done = True
        return done


    # Initialize Environment
    env = EAGERxEnv(name='rx', rate=rate, graph=graph, bridge=bridge, is_done_fn=is_done_fn, reward_fn=reward_fn)
    env = NormalizeActions(Flatten(env))


    # Use stable-baselines
    parent_folder = os.path.dirname(os.path.realpath(__file__))
    save_path = '{}/pendulum'.format(parent_folder)
    model = sb.SAC.load(save_path, env=env)
    # model = sb.PPO("MlpPolicy", env, verbose=1)
    # model = sb.SAC("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=int(180*rate))
    # model.save(save_path)

    rospy.loginfo('Start Evaluation!')
    obs = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

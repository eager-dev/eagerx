#!/usr/bin/env python3

# ROS packages required
# import os
import rospy
import numpy as np
from eagerx_core.core import RxObject, RxGraph, RxBridge, EAGERxEnv
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten

import stable_baselines3 as sb

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define rate (depends on rate of ode)
    rate = 30

    graph = RxGraph.create()
    # graph.load('{}/example.graph'.format(os.path.dirname(os.path.realpath(__file__))))

    pendulum = RxObject.create('pendulum', 'eagerx_bridge_ode', 'pendulum')
    graph.add(pendulum)
    graph.connect(action='action', target=('pendulum', 'actuators', 'action'))
    graph.connect(source=('pendulum', 'sensors', 'observation'), observation='observation')

    graph.gui()

    # Define bridge
    bridge = RxBridge.create('eagerx_bridge_ode', 'bridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.ENVIRONMENT)
    # bridge = RxBridge.create('eagerx_bridge_reality', 'bridge', rate=rate, is_reactive=False, process=process.NEW_PROCESS)

    def reward_fn(prev_obs, obs, action, steps):
        angle = prev_obs['observation'][0][0]
        velocity = prev_obs['observation'][0][1]
        voltage = action['action'][0]
        return - (0.001 * voltage ** 2 + angle ** 2 + 0.001 * (velocity / (1 + abs(angle))) ** 2)

    # Initialize Environment
    env = EAGERxEnv(
        name='rx', rate=rate, graph=graph, bridge=bridge,
        is_done_fn=lambda obs, action, steps: steps > 500 or np.allclose(obs['observation'], 0, atol=0.1),
        reward_fn=reward_fn,
    )
    env = Flatten(env)

    # Use stable-baselines
    model = sb.PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=int(2000*rate*200/20))

    # First reset
    done = False
    obs = env.reset()
    for j in range(20000):
        print('\n[Episode %s]' % j)
        while not done:
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
        obs = env.reset()
        done = False
    print('\n[Finished]')
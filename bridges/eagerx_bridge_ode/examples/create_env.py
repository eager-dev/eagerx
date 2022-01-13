#!/usr/bin/env python3

# ROS packages required
import os
import rospy
from eagerx_core.core import RxObject, RxGraph, RxBridge, EAGERxEnv
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process

import stable_baselines3 as sb

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.DEBUG)

    # Define rate (depends on rate of ode)
    rate = 30

    graph = RxGraph.create()
    # graph.load('{}/example.graph'.format(os.path.dirname(os.path.realpath(__file__))))

    # Open gui
    pendulum = RxObject.create('pendulum', 'eagerx_bridge_ode', 'pendulum')
    graph.add(pendulum)
    graph.connect(action='action', target=('pendulum', 'actuators', 'action'))
    graph.connect(source=('pendulum', 'sensors', 'observation'), observation='observation')

    # graph.gui()

    # Define bridge
    bridge = RxBridge.create('eagerx_bridge_ode', 'bridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.NEW_PROCESS)

    # Initialize Environment
    env = EAGERxEnv(name='ode_env', rate=rate, graph=graph, bridge=bridge)

    # Use stable-baselines
    # model = sb.PPO("MlpPolicy", env, verbose=1)
    # model.learn(total_timesteps=int(2000*rate*200/20))

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
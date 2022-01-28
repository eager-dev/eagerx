#!/usr/bin/env python3

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

    # Initialize empty graph
    graph = RxGraph.create()

    # Create mops
    mops = RxObject.create('mops', 'eagerx_dcsc_setups', 'mops')
    graph.add(mops)

    # Create Butterworth filter
    butterworth_filter = RxNode.create('butterworth_filter', 'eagerx_dcsc_setups', 'butterworth_filter', N=2, Wn=13, rate=rate)
    graph.add(butterworth_filter)

    # Connect the nodes
    graph.connect(action='action', target=('butterworth_filter', 'inputs', 'signal'))
    graph.connect(source=('butterworth_filter', 'outputs', 'filtered'), target=('mops', 'actuators', 'mops_input'))
    graph.connect(source=('mops', 'sensors', 'mops_output'), observation='observation', window=1)
    graph.connect(source=('mops', 'sensors', 'action_applied'), observation='action_applied', window=1)
    graph.render(source=('mops', 'sensors', 'image'), rate=20, display=True)

    # Show in the gui
    graph.gui()

    # Define bridges
    bridge_ode = RxBridge.create('eagerx_bridge_ode', 'bridge', rate=rate, is_reactive=True, process=process.NEW_PROCESS)
    bridge_real = RxBridge.create('eagerx_bridge_real', 'bridge', rate=rate, is_reactive=True, process=process.NEW_PROCESS)

    # Reward and done functions
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
        return steps > 500

    # Initialize Environment
    simulation_env = Flatten(EAGERxEnv(name='ode', rate=rate, graph=graph, bridge=bridge_ode, is_done_fn=is_done_fn, reward_fn=reward_fn))
    real_env = Flatten(EAGERxEnv(name='real', rate=rate, graph=graph, bridge=bridge_real, is_done_fn=is_done_fn, reward_fn=reward_fn))

    # Initialize learner (kudos to Antonin)
    model =  sb.SAC("MlpPolicy", simulation_env, verbose=1)

    # First train in simulation
    simulation_env.render('human')
    model.learn(total_timesteps=int(300*rate))
    simulation_env.close()

    # Evaluate for 30 seconds in simulation
    obs = simulation_env.reset()
    for i in range(int(30 * rate)):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = simulation_env.step(action)
        if done:
            obs = simulation_env.reset()

    model.save('simulation')

    # Train on real system
    model = sb.SAC.load('simulation', env=real_env, ent_coef="auto_0.1")
    real_env.render('human')
    model.learn(total_timesteps=int(420*rate))
    model.save('real')

    # Evaluate on real system
    rospy.loginfo('Start Evaluation!')
    obs = real_env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = real_env.step(action)
        real_env.render()
        if done:
            obs = real_env.reset()

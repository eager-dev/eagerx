#!/usr/bin/env python3

# ROS packages required
from eagerx.core import Object, Bridge, Node, initialize, log, process
initialize('eagerx_core', anonymous=True, log_level=log.INFO)

# Environment
from eagerx.core.rxenv import EAGERxEnv
from eagerx.core.rxgraph import RxGraph
from eagerx.wrappers.flatten import Flatten

# Implementation specific
import eagerx.nodes             # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx.bridges.ode       # Registers OdeBridge # noqa # pylint: disable=unused-import
import eagerx_dcsc_setups.mops  # Registers Mops # noqa # pylint: disable=unused-import

# Other
import numpy as np
import stable_baselines3 as sb

if __name__ == '__main__':
    # Define rate (depends on rate of ode)
    rate = 30.

    # Initialize empty graph
    graph = RxGraph.create()

    # Create mops
    mops = Object.make('Mops', 'mops', render_shape=[480, 480], sensors=['mops_output', 'action_applied'], states=['model_state', 'model_parameters'])
    graph.add(mops)

    # Create Butterworth filter
    butterworth_filter = Node.make('ButterworthFilter', name='bf', rate=rate, N=2, Wn=13, process=process.NEW_PROCESS)
    graph.add(butterworth_filter)

    # Connect the nodes
    graph.connect(action='action', target=('bf', 'inputs', 'signal'))
    graph.connect(source=('bf', 'outputs', 'filtered'), target=('mops', 'actuators', 'mops_input'))
    # graph.connect(action='action', target=('mops', 'actuators', 'mops_input'))
    graph.connect(source=('mops', 'sensors', 'mops_output'), observation='observation', window=1)
    graph.connect(source=('mops', 'sensors', 'action_applied'), observation='action_applied', window=1)

    # Add rendering
    # graph.add_component('mops', 'sensors', 'image')
    # graph.render(source=('mops', 'sensors', 'image'), rate=10, display=True)

    # Show in the gui
    # graph.gui()

    # Define bridges
    bridge = Bridge.make('OdeBridge',   rate=rate, is_reactive=True,  real_time_factor=2, process=process.NEW_PROCESS)

    # Define step function
    def step_fn(prev_obs, obs, action, steps):
        # Calculate reward
        if len(obs['observation'][0]) == 3:
            sin_th, cos_th, thdot = obs['observation'][0]
        else:
            sin_th, cos_th, thdot = 0, -1, 0
        th = np.arctan2(sin_th, cos_th)
        cost = th ** 2 + 0.1 * (thdot / (1 + 10 * abs(th))) ** 2
        # Determine done flag
        done = steps > 500
        # Set info:
        info = dict()
        return obs, -cost, done, info

    # Initialize Environment
    env = Flatten(EAGERxEnv(name='rx', rate=rate, graph=graph, bridge=bridge, step_fn=step_fn))

    # Initialize learner (kudos to Antonin)
    model = sb.SAC("MlpPolicy", env, verbose=1)

    # First train in simulation
    env.render('human')
    # model.learn(total_timesteps=int(300*rate))

    # Evaluate for 30 seconds in simulation
    obs = env.reset()
    eps = 0
    action = env.action_space.sample()
    print(f'Episode {eps}')
    for i in range(int(50000 * rate)):
        # action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        # if i % 500 == 0:
        #     eps += 1
        #     obs = env.reset()
            print(f'Episode {eps}')

    model.save('simulation')
#!/usr/bin/env python3

# EAGERx imports
from eagerx import Object, Bridge, initialize, log, process
initialize('eagerx_core', anonymous=True, log_level=log.INFO)

# Environment imports
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.bridges.openai_gym as eagerx_gym

import stable_baselines3 as sb

if __name__ == '__main__':

    # Define rate (depends on rate of gym env)
    rate = 20

    # Define object
    gym_id = 'Pendulum-v1'  # 'Pendulum-v1', 'Acrobot-v1', 'CartPole-v1', 'MountainCarContinuous-v0'
    name = gym_id.split('-')[0]
    obj = Object.make('GymObject', name, gym_env_id=gym_id, gym_rate=rate, default_action=[0.0], render_shape=[300, 300])

    # Define graph
    graph = Graph.create(objects=[obj])
    graph.connect(source=(name, 'sensors', 'observation'), observation='observation', window=1)
    graph.connect(source=(name, 'sensors', 'reward'), observation='reward', window=1)
    graph.connect(source=(name, 'sensors', 'done'), observation='done', window=1)
    graph.connect(action='action', target=(name, 'actuators', 'action'), window=1)

    # Add rendering
    # graph.add_component(name, 'sensors', 'image')
    # graph.render(source=(name, 'sensors', 'image'), rate=10, display=True)

    # Open gui
    graph.gui()

    # Test save & load functionality
    graph.save('./test.graph')
    graph.load('./test.graph')

    # Define bridge
    bridge = Bridge.make('GymBridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.NEW_PROCESS)

    # Initialize Environment
    env = eagerx_gym.EagerGym(name='rx', rate=rate, graph=graph, bridge=bridge)

    # Turn on rendering
    env.render(mode='human')

    # Use stable-baselines
    # model = sb.PPO("MlpPolicy", env, verbose=1)
    model = sb.SAC("MlpPolicy", env, verbose=1, ent_coef=0.1)
    model.learn(total_timesteps=int(2000*rate*200/20))

    # First reset
    done = False
    obs = env.reset()

    action = env.action_space.sample()
    for j in range(20000):
        print('\n[Episode %s]' % j)
        iter = 0
        while not done:# and iter < 10:
            iter += 1
            obs, reward, done, info = env.step(action)
        obs = env.reset()
        done = False
    print('\n[Finished]')
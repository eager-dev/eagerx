#!/usr/bin/env python3

# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, RxGraph
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_bridge_openai_gym.env import EAGERxGym

import stable_baselines3 as sb
import gym

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define rate (depends on rate of gym env)
    rate = 20

    # Define object
    gym_id = 'Pendulum-v1'
    # gym_id = 'Acrobot-v1'
    # gym_id = 'CartPole-v1'
    # gym_id = 'MountainCarContinuous-v0'
    name = gym_id.split('-')[0]
    obj = RxObject.create(name, 'eagerx_bridge_openai_gym', 'env_object', gym_id=gym_id, rate=rate, sensors=['observation', 'reward', 'done'], zero_action=[0])

    # Define graph
    graph = RxGraph.create(objects=[obj])
    graph.connect(source=(obj.name, 'sensors', 'observation'), observation='observation', delay=0.0, window=1)
    graph.connect(source=(obj.name, 'sensors', 'reward'), observation='reward', window=1)
    graph.connect(source=(obj.name, 'sensors', 'done'), observation='done', window=1)
    graph.connect(action='action', target=(obj.name, 'actuators', 'action'), delay=0.0)

    # Add rendering
    # graph.add_component(obj.name, 'sensors', 'image')
    # graph.render(source=(obj.name, 'sensors', 'image'), rate=10, display=False)

    # Open gui
    # graph.gui()

    # Test save & load functionality
    graph.save('./test.graph')
    graph.load('./test.graph')

    # Define bridge
    bridge = RxBridge.create('eagerx_bridge_openai_gym', 'bridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.NEW_PROCESS)

    # Initialize Environment
    env = EAGERxGym(name='rx', rate=rate, graph=graph, bridge=bridge)

    # Use stable-baselines
    model = sb.PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=int(2000*rate*200/20))

    # First reset
    done = False
    obs = env.reset()
    env.render(mode='human')
    action = env.action_space.sample()
    for j in range(20000):
        print('\n[Episode %s]' % j)
        while not done:
            # print(obs)

            obs, reward, done, info = env.step(action)
            # rgb = env.render(mode='rgb_array')
        obs = env.reset()
        done = False
    print('\n[Finished]')
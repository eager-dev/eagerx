#!/usr/bin/env python3

# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv, RxGraph
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten
from eagerx_bridge_openai_gym.wrappers import RemoveRewardDoneObservation

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
    obj = RxObject.create(name, 'eagerx_bridge_openai_gym', 'env_object', gym_id=gym_id, rate=rate, sensors=['observation', 'reward', 'done'])

    # Define graph
    graph = RxGraph.create(objects=[obj])
    graph.connect(source=(obj.name, 'sensors', 'observation'), observation='observation')
    graph.connect(source=(obj.name, 'sensors', 'reward'), observation='reward')
    graph.connect(source=(obj.name, 'sensors', 'done'), observation='done')
    graph.connect(action='action', target=(obj.name, 'actuators', 'action'))

    # Add rendering
    # graph.add_component(obj.name, 'sensors', 'image')
    # graph.render(source=(obj.name, 'sensors', 'image'), rate=10, display=False)

    # Open gui
    graph.gui()

    # Test save & load functionality
    graph.save('./test.graph')
    graph.load('./test.graph')

    # Define bridge
    bridge = RxBridge.create('eagerx_bridge_openai_gym', 'bridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.ENVIRONMENT)

    # Initialize Environment
    env = EAGERxEnv(name='rx', rate=rate, graph=graph, bridge=bridge,
                    reward_fn=lambda prev_obs, obs, action, steps: obs['reward'][0],
                    is_done_fn=lambda obs, action, steps: obs['done'][0],
                    )
    env = RemoveRewardDoneObservation(env)
    env = Flatten(env)

    # Use stable-baselines
    # env = gym.make(gym_id)
    # model = sb.SAC("MlpPolicy", env, verbose=1)
    # model.learn(total_timesteps=int(200*rate*200/20))

    # First reset
    done = False
    obs = env.reset()
    env.render(mode='human')
    for j in range(20000):
        print('\n[Episode %s]' % j)
        steps = 0
        while not done:
            steps += 1
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            # rgb = env.render(mode='rgb_array')
        print(steps)
        obs = env.reset()
        done = False
    print('\n[Finished]')
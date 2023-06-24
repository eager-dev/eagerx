import eagerx
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from time import time


def run(LOG_DIR, rate, sync, rtf, num_eps, num_steps, actions):
    eagerx.set_log_level(eagerx.DEBUG)

    # Initialize empty graph
    graph = eagerx.Graph.create()

    # Define object
    from eagerx.engines.openai_gym.objects import GymObject
    # todo: Important!!! For this demo to work, make sure to set the env reset state is the same every time.
    gym_id = "Pendulum-v1"  # 'Pendulum-v1', 'Acrobot-v1', 'CartPole-v1', 'MountainCarContinuous-v0'
    name = gym_id.split("-")[0]
    obj = GymObject.make(name, env_id=gym_id, rate=30, default_action=[0.0], render_shape=[300, 300])
    graph.add(obj)

    # Define graph
    graph.connect(source=obj.sensors.observation, observation="observation", window=1)
    graph.connect(source=obj.sensors.reward, observation="reward", window=1)
    graph.connect(source=obj.sensors.terminated, observation="terminated", window=1)
    graph.connect(source=obj.sensors.truncated, observation="truncated", window=1)
    graph.connect(action="action", target=obj.actuators.action, window=1)

    # Open gui
    # graph.gui()

    # Define environment
    class Env(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend):
            self.steps = 0
            super().__init__(name, rate, graph, engine, backend=backend)

        def step(self, action):
            obs = self._step(action)
            return obs, obs["reward"][0], False, self.steps >= 200, dict()

        def reset(self, seed=None, options=None):
            # Reset steps counter
            self.steps = 0

            # Sample states
            states = self.state_space.sample()
            states["pendulum/model_state"] = np.array([0, 0], dtype="float32")

            # Perform reset
            obs = self._reset(states)
            return obs, info

    # Define backend
    from eagerx.backends.ros1 import Ros1
    backend = Ros1.make()
    # from eagerx.backends.single_process import SingleProcess
    # backend = SingleProcess.make()

    # Define engine
    from eagerx.engines.openai_gym.engine import GymEngine
    engine = GymEngine.make(rate=rate, sync=sync, real_time_factor=rtf, process=eagerx.NEW_PROCESS)

    # Initialize Environment
    from eagerx.wrappers import Flatten
    env = Env("Env", rate, graph, engine, backend)
    # env.gui()
    env = Flatten(env)

    # Create fixed action sequence
    t = np.linspace(0, (num_steps - 1) / rate, num=num_steps)
    observations = np.zeros((num_eps, num_steps, env.observation_space.shape[0] + env.action_space.shape[0]))

    # First reset
    total_time = 0
    for eps in range(num_eps):
        print("\n[Episode %s]" % eps)
        obs, info = env.reset()
        start = time()
        for step in range(num_steps):
            action = actions[step]
            observations[eps, step] = np.concatenate([obs, action])
            obs, reward, terminated, truncated, info = env.step(action)
        stop = time()
        total_time += stop - start
    actual_rt_rate = ((num_eps * num_steps) / total_time) / rate

    mean_obs = np.mean(observations, axis=0)
    std_obs = np.std(observations, axis=0)
    fig, ax = plt.subplots(nrows=3, ncols=1)
    if not isinstance(ax, np.ndarray):
        ax = [ax]
    labels = ["$\\cos(\\theta)$", "$\\sin(\\theta)$", "$\\dot{\\theta}$", "$\\tau$"]
    colors = ["gold", "lime", "violet", "grey"]
    all_handles = []
    for idx, i in enumerate([3, 1, 2]):
        handles = []
        label = labels[i]
        color = colors[i]
        mean = mean_obs[:, i]
        std = std_obs[:, i]
        std_idx = np.where(std > 1e-7)[0]
        if len(std_idx) > 0:
            d = 2
            for j in std_idx:
                obs = observations[:, max(0, j-d):min(num_steps, j+d+1), i]
                print(f"sync={sync} | rtf={rtf} | ratio={len(std_idx)}/{num_steps} | idx={j} | std={std[j]} | set={len(set(observations[:, j, i]))}")
                print(obs)
        else:
            print(f"sync={sync} | rtf={rtf} | ratio={len(std_idx)}/{num_steps}")
        line = ax[idx].plot(t, mean, color=color, label=label)
        handles.append(line[0])
        ax[idx].fill_between(t, mean + std, mean - std, color=color, alpha=0.3)
        ax[idx].set(ylabel=label)
        handles.append(mpatches.Patch(color=color, label="std", alpha=0.3))
        ax[idx].legend(
            handles=handles,
            ncol=2,
            prop={"size": 8},
            loc="lower left",
            fancybox=True,
            shadow=True,
        )
        all_handles += handles
    # ax[-1].legend(handles=handles, ncol=4, prop={'size': 8}, loc='upper center', bbox_to_anchor=(0.5, -0.45), fancybox=True, shadow=True)
    # ax[-1].legend(handles=all_handles, ncol=6, prop={'size': 8}, loc='lower left', fancybox=True, shadow=True)
    ax[-1].set(xlabel="$t (s)$")
    sync = "Sync" if sync else "Async"
    real_time_str = rtf if rtf > 0 else '"fast-as-possible"'

    fig.suptitle(
        f"{sync} with factor={real_time_str} (vs {round(actual_rt_rate, 2)} achieved)",
        fontsize=12,
    )
    # fig.savefig(f"{LOG_DIR}/{sync}_{rtf}.png")
    # plt.show()
    print("\n[Finished]")


if __name__ == "__main__":
    num_eps = 5
    num_steps = 10
    # sync = True
    rtf = 0
    LOG_DIR = "/home/r2ci/Documents/project/EAGERx/ICRA/async"
    actions = [[np.random.random_sample()*4-2] for _ in range(num_steps)]

    for sync in [True, False]:
        for rtf in reversed([0.5, 1.0, 1.5, 2.0, 4.0, 6.0, 0]):
            if rtf == 0 and not sync:
                continue
            run(LOG_DIR, 20, sync, rtf, num_eps, num_steps, actions)

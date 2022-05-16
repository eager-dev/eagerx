# ROS packages required
from eagerx import Object, Engine, initialize, log, process

initialize("eagerx_core", anonymous=True, log_level=log.INFO)

# Environment
from eagerx.core.graph import Graph


# Implementation specific
import eagerx.engines.openai_gym as eagerx_gym


# OTHER
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from time import time


def run(LOG_DIR, sync, rtf, num_eps, num_steps, actions):
    # Define rate (depends on rate of gym env)
    rate = 20
    real_time_factor = rtf

    # Define object
    # todo: Important!!! For this demo to work, make sure to set the env reset state is the same every time.
    gym_id = "Pendulum-v1"  # 'Pendulum-v1', 'Acrobot-v1', 'CartPole-v1', 'MountainCarContinuous-v0'
    name = gym_id.split("-")[0]
    obj = Object.make("GymObject", name, env_id=gym_id, rate=rate, default_action=[0.0], render_shape=[300, 300])

    # Define graph
    graph = Graph.create(objects=[obj])
    graph.connect(source=obj.sensors.observation, observation="observation", window=1)
    graph.connect(source=obj.sensors.reward, observation="reward", window=1)
    graph.connect(source=obj.sensors.done, observation="done", window=1)
    graph.connect(action="action", target=obj.actuators.action, window=1)

    # Add rendering
    # graph.add_component(name, 'sensors', 'image')
    # graph.render(source=(name, 'sensors', 'image'), rate=10, display=True)

    # Open gui
    # graph.gui()

    # Test save & load functionality
    graph.save("./test.graph")
    graph.load("./test.graph")

    # Define engine
    engine = Engine.make("GymEngine", rate=rate, sync=sync, real_time_factor=real_time_factor,
                         process=process.NEW_PROCESS)

    # Define step function
    def step_fn(prev_obs, obs, action, steps):
        return obs, obs["reward"][0], steps >= 200, dict()

    # Initialize Environment
    env = eagerx_gym.EagerxGym(name="rx", rate=rate, graph=graph, engine=engine, step_fn=step_fn)

    # Turn on rendering
    env.render(mode="human")

    # Use stable-baselines
    t = np.linspace(0, (num_steps - 1) / rate, num=num_steps)
    observations = np.zeros((num_eps, num_steps, env.observation_space.shape[0] + env.action_space.shape[0]))

    # First reset
    total_time = 0
    for eps in range(num_eps):
        print("\n[Episode %s]" % eps)
        obs = env.reset()
        start = time()
        for step in range(num_steps):
            action = actions[step]
            observations[eps, step] = np.concatenate([obs, action])
            obs, reward, done, info = env.step(action)
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
    real_time_str = real_time_factor if real_time_factor > 0 else '"fast-as-possible"'

    fig.suptitle(
        f"{sync} with factor={real_time_str} (vs {round(actual_rt_rate, 2)} achieved)",
        fontsize=12,
    )
    fig.savefig(f"{LOG_DIR}/{sync}_{real_time_factor}.png")
    plt.show()
    print("\n[Finished]")


if __name__ == "__main__":
    num_eps = 5
    num_steps = 100
    # sync = True
    rtf = 0
    LOG_DIR = "/home/r2ci/Documents/project/EAGERx/ICRA/async"
    actions = [[np.random.random_sample()*4-2] for _ in range(num_steps)]

    for sync in [True, False]:
        for rtf in [0, 0.5, 1.0, 1.5, 2.0, 4.0, 6.0]:
            if rtf == 0 and not sync:
                continue
            run(LOG_DIR, sync, rtf, num_eps, num_steps, actions)

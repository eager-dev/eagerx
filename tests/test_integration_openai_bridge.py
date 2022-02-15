from eagerx import Object, Bridge
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.bridges.openai_gym as eagerx_gym

import pytest

zero_action = {"Pendulum-v0": [0.0], "Acrobot-v0": 0}


@pytest.mark.parametrize("gym_id", ["Pendulum-v0", "Acrobot-v0"])
@pytest.mark.parametrize("eps", [2])
@pytest.mark.parametrize("is_reactive", [True, False])
@pytest.mark.parametrize("p", [process.NEW_PROCESS, process.ENVIRONMENT])
def test_integration_openai_bridge(gym_id, eps, is_reactive, p):
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.INFO)

    # Define rate (depends on rate of gym env)
    rate = 20
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]
    obj = Object.make(
        "GymObject",
        name,
        gym_env_id=gym_id,
        gym_rate=rate,
        default_action=za,
    )

    # Define graph
    graph = Graph.create(objects=[obj])
    graph.connect(source=(name, "sensors", "observation"), observation="observation", window=1)
    graph.connect(source=(name, "sensors", "reward"), observation="reward", window=1)
    graph.connect(source=(name, "sensors", "done"), observation="done", window=1)
    graph.connect(action="action", target=(name, "actuators", "action"), window=1)

    name = f"{name}_{eps}_{is_reactive}_{p}"
    if not is_reactive:
        rtf = 5
    else:
        rtf = 0

    # Define bridge
    bridge = Bridge.make(
        "GymBridge",
        rate=rate,
        is_reactive=is_reactive,
        real_time_factor=rtf,
        process=p,
    )

    # Initialize Environment
    env = eagerx_gym.EagerGym(name=name, rate=rate, graph=graph, bridge=bridge)

    # First reset
    done = False
    _obs = env.reset()

    # Run for several episodes
    for j in range(eps):
        print("\n[Episode %s]" % j)
        iter = 0
        while not done:  # and iter < 10:
            iter += 1
            action = env.action_space.sample()
            _obs, _reward, done, _info = env.step(action)
        _obs = env.reset()
        done = False
    print("\n[Finished]")

    # Shutdown
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")

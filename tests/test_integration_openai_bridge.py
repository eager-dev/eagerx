from eagerx import Object, Bridge, Node, Processor, SpaceConverter
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.bridges.openai_gym as eagerx_gym
import eagerx.nodes  # noqa: F401
import eagerx.converters  # noqa: F401

import pytest

zero_action = {"Pendulum-v0": [0.0], "Acrobot-v1": 0}
NP = process.NEW_PROCESS
ENV = process.ENVIRONMENT


@pytest.mark.parametrize(
    "gym_id, eps, is_reactive, rtf, p",
    [("Pendulum-v0", 2, True, 0, ENV), ("Pendulum-v0", 2, True, 0, NP), ("Acrobot-v1", 2, True, 0, ENV)],
)
def test_integration_openai_bridge(gym_id, eps, is_reactive, rtf, p):
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.WARN)

    # Define rate (depends on rate of gym env)
    rate = 20
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]

    # Get signature of object
    Object.get_spec("GymObject")

    # Create object
    obj = Object.make(
        "GymObject",
        name,
        gym_env_id=gym_id,
        gym_rate=rate,
        default_action=za,
    )

    # Define graph
    graph = Graph.create(objects=[obj])

    # Add butterworth filter
    get_index = Processor.make("GetIndex_Float32MultiArray", index=0)
    bf = Node.make("ButterworthFilter", name="bf", rate=rate, N=2, Wn=4, process=process.ENVIRONMENT)
    graph.add(bf)
    graph.connect(source=obj.sensors.observation, target=bf.inputs.signal, converter=get_index)
    sc = SpaceConverter.make("Space_Float32MultiArray", [-3], [3], dtype="float32")
    graph.connect(source=bf.outputs.filtered, observation="filtered", converter=sc)

    # Connect gym object
    graph.connect(source=obj.sensors.observation, observation="observation", window=1)
    graph.connect(source=obj.sensors.reward, observation="reward", window=1)
    graph.connect(source=obj.sensors.done, observation="done", window=1)
    graph.connect(action="action", target=obj.actuators.action, window=1)

    name = f"{name}_{eps}_{is_reactive}_{p}"

    # Define bridge
    bridge = Bridge.make(
        "GymBridge",
        rate=rate,
        is_reactive=is_reactive,
        real_time_factor=rtf,
        process=p,
    )

    # Initialize Environment
    env = eagerx_gym.EagerGym(name='openai_bridge', rate=rate, graph=graph, bridge=bridge)

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

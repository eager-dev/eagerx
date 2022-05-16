from eagerx import Object, Engine, Node, Processor, SpaceConverter
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.engines.openai_gym as eagerx_gym
import eagerx.nodes  # noqa: F401
import eagerx.converters  # noqa: F401

import pytest

zero_action = {"Pendulum-v0": [0.0], "Pendulum-v1": [0.0], "Acrobot-v1": 0}
NP = process.NEW_PROCESS
ENV = process.ENVIRONMENT


@pytest.mark.timeout(60)
@pytest.mark.parametrize(
    "gym_id, eps, sync, rtf, p",
    [("Pendulum-v1", 2, True, 0, ENV), ("Pendulum-v1", 2, True, 0, NP), ("Acrobot-v1", 2, True, 0, ENV)],
)
def test_integration_openai_engine(gym_id, eps, sync, rtf, p):
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.WARN)

    # Define rate (depends on rate of gym env)
    rate = 20
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]

    # Get signature of object
    Object.info("GymObject")

    # Create object
    obj = Object.make("GymObject", name, env_id=gym_id, rate=rate, default_action=za)

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

    name = f"{name}_{eps}_{sync}_{p}"

    # Define engine
    obj.gui("GymEngine")
    engine = Engine.make("GymEngine", rate=rate, sync=sync, real_time_factor=rtf, process=p)

    # Initialize Environment
    env = eagerx_gym.EagerxGym(name=name, rate=rate, graph=graph, engine=engine)

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

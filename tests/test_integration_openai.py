
# Implementation specific
import eagerx
import eagerx.engines.openai_gym as eagerx_gym
import tests.test.butterworth_filter  # noqa: F401
import tests.test.processors  # noqa: F401

import pytest

zero_action = {"Pendulum-v0": [0.0], "Pendulum-v1": [0.0], "Acrobot-v1": 0}
NP = eagerx.NEW_PROCESS
ENV = eagerx.ENVIRONMENT


@pytest.mark.timeout(40)
@pytest.mark.parametrize(
    "gym_id, eps, sync, rtf, p",
    [("Pendulum-v1", 2, True, 0, ENV), ("Pendulum-v1", 2, True, 0, NP), ("Acrobot-v1", 2, True, 0, ENV)],
)
def test_integration_openai_engine(gym_id, eps, sync, rtf, p):
    eagerx.set_log_level(eagerx.WARN)

    # Define rate (depends on rate of gym env)
    rate = 20
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]

    # Create object
    from eagerx.engines.openai_gym.objects import GymObject
    GymObject.info()
    obj = GymObject.make(name, env_id=gym_id, rate=rate, default_action=za)

    # Define graph
    graph = eagerx.Graph.create(objects=[obj])

    # Add butterworth filter
    from tests.test.processors import GetIndex
    get_index = GetIndex.make(index=0)
    from tests.test.butterworth_filter import ButterworthFilter
    bf = ButterworthFilter.make(name="bf", rate=rate, N=2, Wn=4, process=eagerx.ENVIRONMENT)
    graph.add(bf)
    graph.connect(source=obj.sensors.observation, target=bf.inputs.signal, processor=get_index)
    graph.connect(source=bf.outputs.filtered, observation="filtered")

    # Connect gym object
    graph.connect(source=obj.sensors.observation, observation="observation", window=1)
    graph.connect(source=obj.sensors.reward, observation="reward", window=1)
    graph.connect(source=obj.sensors.done, observation="done", window=1)
    graph.connect(action="action", target=obj.actuators.action, window=1)

    name = f"{name}_{eps}_{sync}_{p}"

    # Define engine
    from eagerx.engines.openai_gym.engine import GymEngine
    obj.gui(GymEngine)
    engine = GymEngine.make(rate=rate, sync=sync, real_time_factor=rtf, process=p)

    # Define backend
    from eagerx.backends.ros1 import Ros1
    backend = Ros1.make()
    # from eagerx.backends.single_process import SingleProcess
    # backend = SingleProcess.make()

    # Initialize Environment
    env = eagerx_gym.EagerxGym(name=name, rate=rate, graph=graph, engine=engine, backend=backend)

    # First reset
    _done = False
    _obs = env.reset()

    # Run for several episodes
    for j in range(eps):
        print("\n[Episode %s]" % j)
        iter = 0
        while iter < 30:  # and iter < 10:
            iter += 1
            action = env.action_space.sample()
            _obs, _reward, _done, _info = env.step(action)
        _obs = env.reset()
        _done = False
    print("\n[Finished]")
    env.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    # for _ in range(100):
    #     test_integration_openai_engine("Pendulum-v1", 20, True, 0, 0)
    test_integration_openai_engine("Acrobot-v1", 2, True, 0, ENV)
    test_integration_openai_engine("Pendulum-v1", 2, True, 0, ENV)
    test_integration_openai_engine("Pendulum-v1", 2, True, 0, NP)

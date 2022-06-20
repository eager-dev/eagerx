
# Implementation specific
import eagerx
import eagerx.engines.openai_gym as eagerx_gym
import tests.test.butterworth_filter  # noqa: F401
import tests.test.processors  # noqa: F401

import pytest

zero_action = {"Pendulum-v0": [0.0], "Pendulum-v1": [0.0], "Acrobot-v1": 0}


@pytest.mark.timeout(40)
@pytest.mark.parametrize("colab", [(True,), (False,)])
def test_render(colab: bool):
    # Start virtual display
    from pyvirtualdisplay import Display
    display = Display(visible=False, backend="xvfb")
    display.start()

    eagerx.set_log_level(eagerx.WARN)

    # Define rate (depends on rate of gym env)
    rate = 20
    gym_id = "Pendulum-v1"
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]

    # Create object
    obj = eagerx.Object.make("GymObject", name, env_id=gym_id, rate=rate, default_action=za)
    obj.config.sensors.append("image")

    # Define graph
    graph = eagerx.Graph.create(objects=obj)

    # Add butterworth filter
    get_index = eagerx.Processor.make("GetIndex", index=0)
    bf = eagerx.Node.make("ButterworthFilter", name="bf", rate=rate, N=2, Wn=4, process=eagerx.ENVIRONMENT)
    graph.add(bf)
    graph.connect(source=obj.sensors.observation, target=bf.inputs.signal, processor=get_index)
    graph.connect(source=bf.outputs.filtered, observation="filtered")

    # Connect gym object
    graph.connect(source=obj.sensors.observation, observation="observation", window=1)
    graph.connect(source=obj.sensors.reward, observation="reward", window=1)
    graph.connect(source=obj.sensors.done, observation="done", window=1)
    graph.connect(action="action", target=obj.actuators.action, window=1)

    # Add rendering
    if not colab:
        graph.render(obj.sensors.image, rate=rate)
    else:
        graph.render(obj.sensors.image, rate=rate, entity_id="ColabRender", process=eagerx.ENVIRONMENT)

    # Define engine
    obj.gui("GymEngine")
    engine = eagerx.Engine.make("GymEngine", rate=rate, sync=True, real_time_factor=0, process=eagerx.NEW_PROCESS)

    # Make backend
    from eagerx.core.ros1 import Ros1
    backend = Ros1.spec()

    # Initialize Environment
    env = eagerx_gym.EagerxGym(name="test_render", rate=rate, graph=graph, engine=engine, backend=backend)

    # First reset
    _obs = env.reset()

    # Run for several episodes
    for j in range(2):
        print("\n[Episode %s]" % j)
        iter = 0
        env.render()
        while iter < 30:  # and iter < 10:
            iter += 1
            if iter == 14:  # Close render window
                env.close()
            print(f"[eps={j}][iter={iter}]")
            action = env.action_space.sample()
            _obs, _reward, _done, _info = env.step(action)
        _obs = env.reset()

    print("\n[Finished]")
    env.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    # TODO: FIX RENDER BLOCKING....
    for i in range(30):
        colab = bool(i % 2)
        print(f"ARE WE USING COLAB? {colab}")
        test_render(colab=colab)
    # test_render(colab=True)


# Implementation specific
import eagerx
import pytest

zero_action = {"Pendulum-v0": [0.0], "Pendulum-v1": [0.0], "Acrobot-v1": 0}


@pytest.mark.timeout(40)
@pytest.mark.parametrize("colab", [(True,), (False,)])
def test_render(colab: bool):
    # Start virtual display
    from pyvirtualdisplay import Display
    display = Display(visible=False, backend="xvfb")
    display.start()

    eagerx.set_log_level(eagerx.DEBUG)

    # Define rate (depends on rate of gym env)
    rate = 20
    gym_id = "Pendulum-v1"
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]

    # Create object
    from eagerx.engines.openai_gym.objects import GymObject
    obj = GymObject.make(name, env_id=gym_id, rate=rate, default_action=za)
    obj.config.sensors.append("image")

    # Define graph
    graph = eagerx.Graph.create(objects=obj)

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

    # Add rendering
    if not colab:
        graph.render(obj.sensors.image, rate=rate)
    else:
        from eagerx.core.nodes import ColabRender
        graph.render(obj.sensors.image, rate=rate, render_cls=ColabRender, process=eagerx.ENVIRONMENT)

    # Define engine
    from eagerx.engines.openai_gym.engine import GymEngine
    obj.gui(GymEngine)
    engine = GymEngine.make(rate=rate, sync=True, real_time_factor=0, process=eagerx.NEW_PROCESS)

    # Make backend
    from eagerx.backends.ros1 import Ros1
    backend = Ros1.make()
    # from eagerx.backends.single_process import SingleProcess
    # backend = SingleProcess.make()

    # Initialize Environment
    import eagerx.engines.openai_gym as eagerx_gym
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
    for i in range(30):
        colab = bool(i % 2)
        print(f"ARE WE USING COLAB? {colab}")
        test_render(colab=colab)
    # test_render(colab=True)

import eagerx

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import
import pytest


@pytest.mark.timeout(20)
@pytest.mark.parametrize("force_start", [True, True, False])
def test_skip_observation(force_start):
    eagerx.set_log_level(eagerx.WARN)

    # todo: rendering requires NEW_PROCESS.
    # todo: implement on_shutdown()
    # todo: check latch (in or outside of condition)?
    # todo: scheduling required with condition? --> separate registration and queue lock?
    # todo: free resources when backend shutdown
    # todo: check dtypes of published messages?
    # todo: see if certain functions can be standardized (e.g. spin, signal shutdown, etc...).

    # Define object
    from tests.test.objects import Arm
    arm = Arm.spec(name="obj", actuators=["ref_vel"], sensors=["N6"], states=["N9"])

    # Define graph
    graph = eagerx.Graph.create(objects=[arm])

    # Create mean-average filter
    N1 = eagerx.Node.make("Process", "N1", rate=1.0, inputs=["in_1"], outputs=["out_1", "out_2"], process=eagerx.NEW_PROCESS)
    graph.add(N1)

    # Connect sensors (= outputs of object)
    graph.connect(action="act_1", target=N1.inputs.in_1)
    graph.connect(source=N1.outputs.out_1, target=arm.actuators.ref_vel)
    graph.connect(source=N1.outputs.out_2, observation="maf_state", skip=True, window=1)
    graph.connect(source=arm.sensors.N6, observation="sens_1")

    # Define engine
    engine = eagerx.Engine.make("TestEngine", rate=20, sync=True, real_time_factor=0, process=eagerx.NEW_PROCESS)

    # Make backend - SingleProcess breaks Assertion fail.
    from eagerx.backends.ros1 import Ros1
    backend = Ros1.spec()

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend, force_start):
            super().__init__(name, rate, graph, engine, backend=backend, force_start=force_start)

        def step(self, action):
            obs = self._step(action)
            return obs, 0, False, {}

        def reset(self):
            states = self.state_space.sample()
            obs = self._reset(states)
            return obs

    # Initialize Environment
    try:
        env = TestEnv("rx", 20, graph, engine, backend, force_start=force_start)
        msg = "We should have exited the environment here. IMPORTANT!!! The test with force_start=False cannot be the first test."
        assert force_start, msg
    except eagerx.core.constants.BackendException:
        if not force_start:
            print("Successfully exited environment initialization, because environment already exists and force_start=False")
            return
        else:
            raise

    # First reset
    import time
    N = 50
    eps = 2
    _ = env.reset()
    env.render(mode="human")
    action = env.action_space.sample()
    for j in range(eps):
        print("\n[Episode %s]" % j)
        start = time.time()
        for i in range(N):
            _ = env.step(action)
            env.render(mode="rgb_array")
        print(f"TIME: {N/(time.time() - start)}")
        env.reset()
    print("\n[Finished]")


if __name__ == "__main__":
    for i in range(30):
        print(f"ITERATION {i}")
        test_skip_observation(True)
    test_skip_observation(True)
    test_skip_observation(True)
    test_skip_observation(False)

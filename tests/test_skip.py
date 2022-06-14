import eagerx

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import
import pytest


@pytest.mark.timeout(20)
@pytest.mark.parametrize("force_start", [True, True, False])
def test_skip_observation(force_start):
    # Define object
    arm = eagerx.Object.make("Arm", "obj", actuators=["ref_vel"], sensors=["N6"], states=["N9"])

    # Define graph
    graph = eagerx.Graph.create(objects=[arm])

    # Create mean-average filter
    N1 = eagerx.Node.make("Process", "N1", rate=1.0, inputs=["in_1"], outputs=["out_1", "out_2"])
    graph.add(N1)

    # Connect sensors (= outputs of object)
    graph.connect(action="act_1", target=N1.inputs.in_1)
    graph.connect(source=N1.outputs.out_1, target=arm.actuators.ref_vel)
    graph.connect(source=N1.outputs.out_2, observation="maf_state", skip=True, window=1)
    graph.connect(source=arm.sensors.N6, observation="sens_1")

    # Define engine
    engine = eagerx.Engine.make("TestEngine", rate=20, sync=True, real_time_factor=0,
                                process=eagerx.process.ENVIRONMENT)

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, force_start):
            super().__init__(name, rate, graph, engine, force_start=force_start)

        def step(self, action):
            obs = self._step(action)
            return obs, 0, False, {}

        def reset(self):
            states = self.state_space.sample()
            obs = self._reset(states)
            return obs

    # Initialize Environment
    try:
        env = TestEnv(name="rx", rate=7, graph=graph, engine=engine, force_start=force_start)
        msg = "We should have exited the environment here. IMPORTANT!!! The test with force_start=False cannot be the first test."
        assert force_start, msg
    except eagerx.core.constants.BackendException:
        if not force_start:
            print("Successfully exited environment initialization, because environment already exists and force_start=False")
            return
        else:
            raise

    # First reset
    _ = env.reset()
    env.render(mode="human")
    action = env.action_space.sample()
    for j in range(2):
        print("\n[Episode %s]" % j)
        for i in range(50):
            _ = env.step(action)
            env.render(mode="rgb_array")
        env.reset()
    print("\n[Finished]")

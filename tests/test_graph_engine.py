import eagerx

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import

import pytest


@pytest.mark.timeout(20)
def test_graph_engine():
    eagerx.set_log_level(eagerx.WARN)

    # Define object
    arm = eagerx.Object.make("Arm", "obj", actuators=["N8"], sensors=["N6"], states=["N9"])

    # Define graph
    graph = eagerx.Graph.create(objects=[arm])

    # Connect sensors (= outputs of object)
    graph.connect(source=arm.sensors.N6, observation="obs_1")
    graph.connect(action="act_1", target=arm.actuators.N8)

    # Define engine
    engine = eagerx.Engine.make("TestEngine", rate=20, sync=True, real_time_factor=0, process=eagerx.ENVIRONMENT)

    # Define backend
    from eagerx.core.ros1 import Ros1
    backend = Ros1.spec()

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend):
            super().__init__(name, rate, graph, engine, backend, force_start=True)

        def step(self, action):
            obs = self._step(action)
            return obs, 0, False, {}

        def reset(self):
            states = self.state_space.sample()
            obs = self._reset(states)
            return obs

    # Initialize Environment
    env = TestEnv(name="graph_engine", rate=7, graph=graph, engine=engine, backend=backend)

    # First reset
    env.reset()
    env.render(mode="human")
    action = env.action_space.sample()
    for j in range(2):
        print("\n[Episode %s]" % j)
        for i in range(50):
            env.step(action)
            env.render(mode="rgb_array")
        env.reset()
    print("\n[Finished]")

    # Shutdown test
    env.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    test_graph_engine()

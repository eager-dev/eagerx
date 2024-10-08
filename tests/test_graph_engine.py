import eagerx

import pytest


@pytest.mark.timeout(20)
def test_graph_engine():
    eagerx.set_log_level(eagerx.WARN)

    # Define object
    from tests.test.objects import Arm
    arm = Arm.make("obj", actuators=["N8"], sensors=["N6"], states=["N9"])

    # Define graph
    graph = eagerx.Graph.create(objects=[arm])

    # Connect sensors (= outputs of object)
    graph.connect(source=arm.sensors.N6, observation="obs_1")
    graph.connect(action="act_1", target=arm.actuators.N8)

    # Define engine
    from tests.test.engine import TestEngine
    engine = TestEngine.make(rate=20, sync=True, real_time_factor=0, process=eagerx.ENVIRONMENT)

    # Define backend
    # from eagerx.backends.ros1 import Ros1
    # backend = Ros1.make()
    from eagerx.backends.single_process import SingleProcess
    backend = SingleProcess.make()

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend, render_mode=None):
            super().__init__(name, rate, graph, engine, backend, force_start=True, render_mode=render_mode)

        def step(self, action):
            obs = self._step(action)
            # Render
            if self.render_mode == "human":
                self.render()
            return obs, 0, False, False, {}

        def reset(self, seed=None, options=None):
            states = self.state_space.sample()
            obs = self._reset(states)
            # Render
            if self.render_mode == "human":
                self.render()
            return obs, {}

    # Initialize Environment
    env = TestEnv(name="graph_engine", rate=7, graph=graph, engine=engine, backend=backend, render_mode="human")

    # First reset
    env.reset()
    action = env.action_space.sample()
    for j in range(2):
        print("\n[Episode %s]" % j)
        for i in range(50):
            env.step(action)
        env.reset()
    print("\n[Finished]")

    # Shutdown test
    env.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    test_graph_engine()

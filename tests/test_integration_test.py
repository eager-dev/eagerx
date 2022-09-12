import eagerx

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import

import pytest

NP = eagerx.NEW_PROCESS
ENV = eagerx.ENVIRONMENT


@pytest.mark.timeout(40)
@pytest.mark.parametrize(
    "eps, steps, sync, rtf, p",
    [(3, 3, True, 0, ENV), (20, 40, True, 0, NP), (3, 3, True, 4, NP), (20, 40, False, 4, NP), (3, 3, False, 4, ENV)],
)
def test_integration_test_engine(eps, steps, sync, rtf, p):
    # Start roscore
    eagerx.set_log_level(eagerx.WARN)

    # Define unique name for test environment
    name = f"{eps}_{steps}_{sync}_{p}"
    node_p = p
    engine_p = p
    rate = 17

    # Define nodes
    from tests.test.nodes import ProcessNode, KalmanNode, RealResetNode
    N1 = ProcessNode.make("N1", rate=18, process=node_p, inputs=["in_1", "in_2"], outputs=["out_1"])
    KF = KalmanNode.make("KF", rate=19, process=node_p, inputs=["in_1", "in_2"], outputs=["out_1", "out_2"])
    N3 = RealResetNode.make("N3", rate=rate, process=node_p, inputs=["in_1"], targets=["target_1"])

    # Define object
    from tests.test.objects import Viper
    viper = Viper.make("obj", actuators=["N8"], sensors=["N6", "N7"], states=["N9"])

    # Define graph
    graph = eagerx.Graph.create(nodes=[N1, N3, KF], objects=[viper])

    # Connect outputs KF
    graph.connect(source=KF.outputs.out_1, observation="obs_2")

    # Connect sensors (= outputs of object)
    graph.connect(source=viper.sensors.N6, observation="obs_1")
    graph.connect(source=viper.sensors.N6, target=N3.inputs.in_1)
    graph.connect(source=viper.sensors.N6, target=KF.inputs.in_1)

    # Connect actions (= outputs of environment)
    graph.connect(action="act_1", target=KF.inputs.in_2, skip=True)
    graph.connect(action="act_1", target=N3.feedthroughs.out_1)
    graph.connect(action="act_2", target=N1.inputs.in_1, skip=True)

    # Connect outputs N1
    graph.connect(source=N1.outputs.out_1, observation="obs_3")
    graph.connect(source=viper.sensors.N7, target=N1.inputs.in_2)

    # Connect outputs & targets N3
    graph.connect(source=N3.outputs.out_1, target=viper.actuators.N8)
    graph.connect(source=viper.states.N9, target=N3.targets.target_1)

    # Define render
    # graph.render(source=viper.sensors.N6, rate=1, processor=eagerx.Processor.make("ToUint8"))

    # Open GUI (only opens if eagerx-gui installed)
    graph.gui()

    # Define engine
    from tests.test.engine import TestEngine
    viper.gui(TestEngine)
    engine = TestEngine.make(rate=20, sync=sync, real_time_factor=rtf, process=engine_p)

    # Define backend
    from eagerx.backends.ros1 import Ros1
    backend = Ros1.make()
    # from eagerx.backends.single_process import SingleProcess
    # backend = SingleProcess.make()

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend):
            super().__init__(name, rate, graph, engine, backend, force_start=True)

        def step(self, action):
            obs = self._step(action)
            return obs, 0, False, {}

        def reset(self):
            sampled = self.state_space.sample()
            states = {"obj/N9": sampled["obj/N9"], "engine/param_1": sampled["engine/param_1"]}
            obs = self._reset(states)
            return obs

    # Initialize Environment
    env = TestEnv(name=name, rate=rate, graph=graph, engine=engine, backend=backend)

    # First reset
    env.reset()
    env.render(mode="human")
    action = env.action_space.sample()
    for j in range(eps):
        print("\n[Episode %s]" % j)
        for i in range(steps):
            env.step(action)
            env.render(mode="rgb_array")
        env.reset()
    print("\n[Finished]")
    env.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    test_integration_test_engine(20, 40, True, 0, NP)
    test_integration_test_engine(3, 3, True, 0, ENV)
    test_integration_test_engine(20, 40, False, 4, NP)
    test_integration_test_engine(3, 3, False, 4, ENV)

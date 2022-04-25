from eagerx import Object, Bridge, Node, ResetNode, Converter
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
from eagerx.wrappers import Flatten

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import

import pytest

NP = process.NEW_PROCESS
ENV = process.ENVIRONMENT


@pytest.mark.timeout(60)
@pytest.mark.parametrize(
    "eps, steps, sync, rtf, p",
    [(3, 3, True, 0, ENV), (20, 40, True, 0, NP), (3, 3, True, 4, NP), (20, 40, False, 4, NP), (3, 3, False, 4, ENV)],
)
def test_integration_test_bridge(eps, steps, sync, rtf, p):
    # Start roscore
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.WARN)

    # Define unique name for test environment
    name = f"{eps}_{steps}_{sync}_{p}"
    node_p = p
    bridge_p = p
    rate = 17

    # Define nodes
    N1 = Node.make("Process", "N1", rate=18, process=node_p, inputs=["in_1", "in_2"], outputs=["out_1"])
    KF = Node.make("KalmanFilter", "KF", rate=19, process=node_p, inputs=["in_1", "in_2"], outputs=["out_1", "out_2"])
    N3 = ResetNode.make("RealReset", "N3", rate=rate, process=node_p, inputs=["in_1"], targets=["target_1"])

    # Define object
    viper = Object.make("Viper", "obj", actuators=["N8"], sensors=["N6", "N7"], states=["N9"])

    # Define converter (optional)
    RosString_RosUInt64 = Converter.make("RosString_RosUInt64", test_arg="test")
    RosImage_RosUInt64 = Converter.make("RosImage_RosUInt64", test_arg="test")

    # Define graph
    graph = Graph.create(nodes=[N1, N3, KF], objects=[viper])

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
    graph.connect(source=N3.outputs.out_1, target=viper.actuators.N8, converter=RosString_RosUInt64)
    graph.connect(source=viper.states.N9, target=N3.targets.target_1)

    # Define render
    graph.render(source=viper.sensors.N6, rate=1, converter=RosImage_RosUInt64)

    # Open GUI (only opens if eagerx-gui installed)
    graph.gui()

    # Define bridge
    bridge = Bridge.make("TestBridge", rate=20, sync=sync, real_time_factor=rtf, process=bridge_p)

    # Initialize Environment
    env = EagerxEnv(
        name=name,
        rate=rate,
        graph=graph,
        bridge=bridge,
        reset_fn=lambda env: {
            "obj/N9": env.state_space.sample()["obj/N9"],
            "bridge/param_1": env.state_space.sample()["bridge/param_1"],
        },
    )
    env = Flatten(env)

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
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")

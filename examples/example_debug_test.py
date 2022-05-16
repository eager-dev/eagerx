# Environment imports
import eagerx

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import

import time

zero_action = {"Pendulum-v0": [0.0], "Acrobot-v1": 0}


def graph_engine(idx):
    # Start roscore
    roscore = eagerx.initialize("eagerx_core", anonymous=True, log_level=eagerx.log.DEBUG)

    sync = True
    rtf = 0
    p = 0

    # Define unique name for test environment
    node_p = p
    engine_p = p
    rate = 17

    # Define nodes
    N1 = eagerx.Node.make("Process", "N1", rate=rate, process=node_p, inputs=["in_1"], outputs=["out_1"])
    KF = eagerx.Node.make("KalmanFilter", "KF", rate=rate, process=node_p, inputs=["in_1", "in_2"], outputs=["out_1", "out_2"])
    N3 = eagerx.ResetNode.make("RealReset", "N3", rate=rate, process=node_p, inputs=["in_1"], targets=["target_1"])

    # Define object
    viper = eagerx.Object.make("Viper", "obj", actuators=["N8", "N12"], sensors=["N6"], states=["N9"])

    # Define converter (optional)
    RosString_RosUInt64 = eagerx.Converter.make("RosString_RosUInt64", test_arg="test")
    RosImage_RosUInt64 = eagerx.Converter.make("RosImage_RosUInt64", test_arg="test")

    # Add simple identity processor
    # IdentityProcessor = Processor.make("IdentityProcessor")
    # viper.sensors.N6.converter = IdentityProcessor

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

    # Connect outputs & targets N3
    graph.connect(source=N3.outputs.out_1, target=viper.actuators.N8, converter=RosString_RosUInt64)
    graph.connect(source=N3.outputs.out_1, target=viper.actuators.N12)
    graph.connect(source=viper.states.N9, target=N3.targets.target_1)

    # Define render
    graph.render(source=viper.sensors.N6, rate=1, converter=RosImage_RosUInt64)

    # Open GUI (only opens if eagerx-gui installed)
    graph.gui()

    # Define engine
    engine = eagerx.Engine.make("TestEngine", rate=20, sync=sync, real_time_factor=rtf, process=engine_p)

    # Initialize Environment
    name = str(time.time()).replace(".", "_")
    env = eagerx.EagerxEnv(
        name=f"rx_{name}",
        rate=rate,
        graph=graph,
        engine=engine,
        reset_fn=lambda env: {
            "obj/N9": env.state_space.sample()["obj/N9"],
            "engine/param_1": env.state_space.sample()["engine/param_1"],
        },
    )
    env = eagerx.wrappers.Flatten(env)

    # First reset
    env.reset()

    # Shutdown
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    graph_engine(str(time.time()).replace(".", "_"))

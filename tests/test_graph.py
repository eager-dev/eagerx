from eagerx import Object, Bridge, Node, ResetNode, Converter, BaseConverter
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
from eagerx.wrappers import Flatten

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import

import pytest


@pytest.mark.timeout(60)
def test_graph():
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.INFO)
    rate = 7

    # Get info on various specs.
    import eagerx.converters
    eagerx.Processor.info("GetIndex_Float32MultiArray")
    eagerx.SpaceConverter.info("Space_Float32MultiArray")
    eagerx.Object.info("Viper")
    eagerx.Object.info("Viper", method="spec")
    eagerx.Object.info("Viper", method=["spec"])

    # Define nodes
    N0 = Node.make("Process", "N0", rate=rate, process=process.ENVIRONMENT, inputs=["in_1"], outputs=["out_1"])
    N1 = Node.make("Process", "N1", rate=rate, process=process.ENVIRONMENT, inputs=["in_1"], outputs=["out_1"])
    N2 = Node.make("Process", "N2", rate=rate, process=process.ENVIRONMENT, inputs=["in_1"], outputs=["out_1"])
    KF = Node.make("KalmanFilter", "KF", rate=rate, process=process.NEW_PROCESS, inputs=["in_1", "in_2"],
                   outputs=["out_1", "out_2"])
    N3 = ResetNode.make("RealReset", "N3", rate=rate, process=process.NEW_PROCESS, inputs=["in_1", "in_2"],
                        targets=["target_1"])

    # Define object
    viper = Object.make("Viper", "obj", position=[1.0, 1.0, 1.0], actuators=["N8"], sensors=["N6"], states=["N9"])

    # Test SpecView
    with N3.inputs.unlocked:
        _ = N3.inputs.__repr__()
        for _ in N3.inputs:
            pass
    try:
        _ = N3.inputs.dummy_cname
    except AttributeError as e:
        print("Must fail! ", e)

    try:
        N3.inputs.dummy_cname = dict()
    except AttributeError as e:
        print("Must fail! ", e)
    _ = len(N3.inputs)

    # Define converter (optional)
    RosString_RosUInt64 = Converter.make("RosString_RosUInt64", test_arg="test")
    RosImage_RosUInt64 = Converter.make("RosImage_RosUInt64", test_arg="test")

    # Define graphs in different ways
    _ = Graph.create(nodes=N0).__str__()
    _ = N0.inputs.__repr__()
    _ = N0.inputs.__str__()
    try:
        _ = N0.inputs.dummy_cname
    except AttributeError as e:
        print("Must fail! ", e)

    graph = Graph.create(nodes=[N3, KF], objects=viper)

    # Rendering
    graph.render(
        source=viper.sensors.N6,
        rate=1,
        converter=RosImage_RosUInt64,
        display=False,
    )
    graph.render(
        source=viper.sensors.N6,
        rate=1,
        converter=RosImage_RosUInt64,
        display=False,
    )

    # Connect/remove observation
    graph.connect(source=viper.sensors.N6, observation="obs_1", delay=0.0)
    graph.remove_component(observation="obs_1")
    graph.connect(source=viper.sensors.N6, observation="obs_1", delay=0.0)
    graph.disconnect(source=viper.sensors.N6, observation="obs_1", remove=True)

    # Connect/remove action
    graph.connect(action="act_2", target=KF.inputs.in_2, skip=True)
    graph.remove_component(action="act_2")
    graph.connect(action="act_2", target=KF.inputs.in_2, skip=True)
    graph.disconnect(action="act_2", target=KF.inputs.in_2, remove=True)

    # Connect graph
    graph.connect(source=KF.outputs.out_1, observation="obs_3")
    graph.connect(source=viper.sensors.N6, observation="obs_1", delay=0.0)
    graph.connect(source=viper.sensors.N6, target=KF.inputs.in_1, delay=0.0)
    graph.connect(source=viper.sensors.N6, target=N3.inputs.in_1)

    graph.connect(action="act_2", target=KF.inputs.in_2, skip=True)
    graph.connect(action="act_2", target=N3.feedthroughs.out_1, delay=0.0)
    graph.connect(source=viper.states.N9, target=N3.targets.target_1)
    graph.connect(source=N3.outputs.out_1, target=viper.actuators.N8, converter=RosString_RosUInt64)

    # Set & get parameters
    _ = graph.get(N3)
    _ = graph.get(action="act_2", parameter="converter")
    graph.set(graph.get(action="act_2"), action="act_2")
    graph.set(graph.get(observation="obs_1"), observation="obs_1")
    graph.set({"window": 1}, observation="obs_1")
    _ = graph.get(observation="obs_1", parameter="converter")
    _ = graph.get(N3.config, parameter="test_arg")
    _ = graph.get(viper.sensors.N6)
    graph.set("Modified", N3.config, parameter="test_arg")
    graph.set({"test_arg": "Modified"}, N3.config)
    graph.set([1, 1, 1], viper.config, parameter="position")

    # Replace output converter (disconnects all connections (obs_1, KF, N3))
    graph.set({"converter": RosString_RosUInt64}, viper.sensors.N6)
    graph.set({"converter": BaseConverter.make("Identity")}, viper.sensors.N6)
    graph.render(source=viper.sensors.N6, rate=1, converter=RosImage_RosUInt64)  # Reconnect
    graph.connect(source=viper.sensors.N6, observation="obs_1", delay=0.0)  # Reconnect
    graph.connect(source=viper.sensors.N6, target=KF.inputs.in_1, delay=0.0)  # Reconnect
    graph.connect(source=viper.sensors.N6, target=N3.inputs.in_1)  # Reconnect

    # Remove component. For action/observation use graph._remove_action/observation(...) instead.
    graph.remove_component(N3.inputs.in_2)
    graph.add_component(N3.inputs.in_2)
    graph.remove_component(N3.inputs.in_2)

    # Rename action & observation
    graph.rename("act_1", action="act_2")
    graph.rename("act_2", action="act_1")
    graph.rename("act_1", action="act_2")
    graph.rename("obs_2", observation="obs_3")

    # Remove & add action (without action terminal removal)
    graph.disconnect(action="act_1", target=KF.inputs.in_2)
    graph.connect(
        action="act_1",
        target=KF.inputs.in_2,
        converter=None,
        delay=None,
        window=None,
        skip=True,
    )

    # Remove & add observation (with observation terminal removal)
    graph.disconnect(source=viper.sensors.N6, observation="obs_1")
    graph.add_component(observation="obs_1")  # Add input terminal
    graph.connect(
        source=viper.sensors.N6,
        observation="obs_1",
        converter=None,
        delay=None,
        window=None,
    )

    # Remove & add other input
    graph.disconnect(source=viper.sensors.N6, target=KF.inputs.in_1)
    graph.connect(source=viper.sensors.N6, target=KF.inputs.in_1)

    # Works with other sources as well, but then specify "source" instead of "action" as optional arg to connect(..) and disconnect(..).
    # NOTE: with the remove=False flag, we avoid removing terminal 'obs_1'
    graph.disconnect(source=viper.sensors.N6, observation="obs_1", remove=False)

    # GUI routine for making connections
    source = viper.sensors.N6
    target = ("env/observations", "inputs", "obs_1")
    # GUI: Identify if source/target is action/observation
    observation = target[2] if target[0] == "env/observations" else None
    action = source[2] if source()[0] == "env/actions" else None
    params = graph.get(observation="obs_1")  # Grab already defined parameters from input component
    if len(params) == 0:  # If observation, dict will be empty.
        converter = graph.get(source, parameter="space_converter")
        delay, window = 0, 0
    else:  # If not observation, these values will always be present
        converter, delay, window = params["converter"], params["delay"], params["window"]
    # GUI: open dialogue box where users can modify converter, delay, window etc... Use previous params to set initial values.
    # GUI: converter, delay, window = ConnectionOptionsDialogueBox(converter, delay, window)
    target = None if observation else target  # If we have an observation, it will be the target instead in .connect(..)
    source = None if action else source  # If we have an action, it will be the source instead in .connect(..)
    # GUI: use the modified params via the dialogue box to connect.
    graph.connect(
        source=source,
        target=target,
        action=action,
        observation=observation,
        converter=converter,
        delay=delay,
        window=window,
    )

    # Connect N1
    graph.add(N1)
    graph.connect(source=N1.outputs.out_1, observation="obs_6")
    graph.connect(action="act_6", target=N1.inputs.in_1, skip=True)

    # Connect N1
    graph.add(N2)
    graph.connect(source=N2.outputs.out_1, observation="obs_N2")
    graph.connect(action="act_N2", target=N2.inputs.in_1)
    graph.remove(N2, remove=True)

    # Test when KF skips all inputs at t=0
    graph.remove_component(KF.inputs.in_1)

    # Open gui (only opens if eagerx-gui is installed)
    # graph.gui()

    # Test save & load functionality
    graph.save("./test.graph")
    graph.load("./test.graph")

    # Plot
    import matplotlib.pyplot as plt

    plt.ion()
    graph.is_valid(plot=True)

    # Define bridge
    bridge = Bridge.make("TestBridge", rate=20, is_reactive=False, real_time_factor=5.5, process=process.NEW_PROCESS)

    # Initialize Environment
    env = EagerxEnv(
        name="graph",
        rate=rate,
        graph=graph,
        bridge=bridge,
        reset_fn=lambda env: {
            "obj/N9": env.state_space.sample()["obj/N9"],
            "bridge/param_1": env.state_space.sample()["bridge/param_1"],
        },
    )
    env = Flatten(env)

    # Test message broker
    env.env.mb.print_io_status()
    env.env.mb.print_io_status(node_names="/graph/environment")

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

    # Test acyclic graph
    graph.remove_component(action="act_1")
    graph.connect(action="act_1", target=KF.inputs.in_2, skip=False)
    graph.connect(action="act_1", target=N3.feedthroughs.out_1)
    try:
        graph.is_valid(plot=True)
        raise RuntimeError("Cycle not detected.")  # Choose error other than assertion.
    except AssertionError as e:
        print("ALGEBRAIC CYCLES CORRECTLY DETECTED")
        print(e)
        pass

    # Shutdown test
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")

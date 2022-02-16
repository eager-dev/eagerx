from eagerx import Object, Bridge, Node, ResetNode, Converter, BaseConverter
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.env import EagerEnv
from eagerx.core.graph import Graph
from eagerx.wrappers import Flatten

# Implementation specific
import eagerx.bridges.test  # noqa # pylint: disable=unused-import

import pytest


def test_graph():
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.INFO)
    rate = 7

    # Define nodes
    N1 = Node.make(
        "Process",
        "N1",
        rate=rate,
        process=process.ENVIRONMENT,
        inputs=["in_1"],
        outputs=["out_1"],
    )
    KF = Node.make(
        "KalmanFilter",
        "KF",
        rate=rate,
        process=process.NEW_PROCESS,
        inputs=["in_1", "in_2"],
        outputs=["out_1", "out_2"],
    )
    N3 = ResetNode.make(
        "RealReset",
        "N3",
        rate=rate,
        process=process.NEW_PROCESS,
        inputs=["in_1", "in_2"],
        targets=["target_1"],
    )

    # Define object
    viper = Object.make(
        "Viper",
        "obj",
        position=[1.0, 1.0, 1.0],
        actuators=["N8"],
        sensors=["N6"],
        states=["N9"],
    )

    # Define converter (optional)
    RosString_RosUInt64 = Converter.make("RosString_RosUInt64", test_arg="test")
    RosImage_RosUInt64 = Converter.make("RosImage_RosUInt64", test_arg="test")

    # Define graphs in different ways
    _ = Graph.create(nodes=N3).__str__
    graph = Graph.create(nodes=[N3, KF], objects=viper)

    # Rendering
    graph.render(
        source=("obj", "sensors", "N6"),
        rate=1,
        converter=RosImage_RosUInt64,
        display=False,
    )
    graph.render(
        source=("obj", "sensors", "N6"),
        rate=1,
        converter=RosImage_RosUInt64,
        display=False,
    )

    # Connect/remove observation
    graph.connect(source=("obj", "sensors", "N6"), observation="obs_1", delay=0.0)
    graph.remove_component(observation="obs_1")
    graph.connect(source=("obj", "sensors", "N6"), observation="obs_1", delay=0.0)
    graph.disconnect(source=("obj", "sensors", "N6"), observation="obs_1", remove=True)

    # Connect/remove action
    graph.connect(action="act_2", target=("KF", "inputs", "in_2"), skip=True)
    graph.remove_component(action="act_2")
    graph.connect(action="act_2", target=("KF", "inputs", "in_2"), skip=True)
    graph.disconnect(action="act_2", target=("KF", "inputs", "in_2"), remove=True)

    # Connect graph
    graph.connect(source=("KF", "outputs", "out_1"), observation="obs_3")
    graph.connect(source=("obj", "sensors", "N6"), observation="obs_1", delay=0.0)
    graph.connect(source=("obj", "sensors", "N6"), target=("KF", "inputs", "in_1"), delay=0.0)
    graph.connect(source=("obj", "sensors", "N6"), target=("N3", "inputs", "in_1"))

    graph.connect(action="act_2", target=("KF", "inputs", "in_2"), skip=True)
    graph.connect(action="act_2", target=("N3", "feedthroughs", "out_1"), delay=0.0)
    graph.connect(source=("obj", "states", "N9"), target=("N3", "targets", "target_1"))
    graph.connect(source=("N3", "outputs", "out_1"), target=("obj", "actuators", "N8"), converter=RosString_RosUInt64)

    # Set & get parameters
    _ = graph.get_parameter("converter", action="act_2")
    graph.set_parameters(graph.get_parameters(action="act_2"), action="act_2")
    graph.set_parameters(graph.get_parameters(observation="obs_1"), observation="obs_1")
    graph.set_parameter("window", 1, observation="obs_1")
    _ = graph.get_parameter("converter", observation="obs_1")
    _ = graph.get_parameter("test_arg", name="N3")
    _ = graph.get_parameters("obj", "sensors", "N6")
    graph.set_parameter("test_arg", "Modified with set_parameter", name="N3")
    graph.set_parameter("test_arg", "Modified with set_parameter", name="N3")
    graph.set_parameter("position", [1, 1, 1], name="obj")

    # Replace output converter (disconnects all connections (obs_1, KF, N3))
    graph.set_parameter("converter", RosString_RosUInt64, name="obj", component="sensors", cname="N6")
    identity = BaseConverter.make("Identity")
    graph.set_parameter("converter", identity, name="obj", component="sensors", cname="N6")
    graph.render(source=("obj", "sensors", "N6"), rate=1, converter=RosImage_RosUInt64)  # Reconnect
    graph.connect(source=("obj", "sensors", "N6"), observation="obs_1", delay=0.0)  # Reconnect
    graph.connect(source=("obj", "sensors", "N6"), target=("KF", "inputs", "in_1"), delay=0.0)  # Reconnect
    graph.connect(source=("obj", "sensors", "N6"), target=("N3", "inputs", "in_1"))  # Reconnect

    # Remove component. For action/observation use graph._remove_action/observation(...) instead.
    graph.remove_component("N3", "inputs", "in_2")

    # Rename entity (object/node) and all associated connections
    graph.rename("KF", "KF2")
    graph.rename("KF2", "KF")

    # Rename action & observation
    graph.rename("act_2", "act_1", name="env/actions", component="outputs")
    graph.rename("act_1", "act_2", action="act_2")
    graph.rename("act_2", "act_1", action="act_1")
    graph.rename("obs_3", "obs_2", observation="obs_2")

    # Remove & add action (without action terminal removal)
    graph.disconnect(action="act_1", target=("KF", "inputs", "in_2"))
    graph.connect(
        action="act_1",
        target=("KF", "inputs", "in_2"),
        converter=None,
        delay=None,
        window=None,
        skip=True,
    )

    # Remove & add observation (with observation terminal removal)
    graph.disconnect(source=("obj", "sensors", "N6"), observation="obs_1")
    graph.add_component(observation="obs_1")  # Add input terminal
    graph.connect(
        source=("obj", "sensors", "N6"),
        observation="obs_1",
        converter=None,
        delay=None,
        window=None,
    )

    # Remove & add other input
    graph.disconnect(source=("obj", "sensors", "N6"), target=("KF", "inputs", "in_1"))
    graph.connect(source=("obj", "sensors", "N6"), target=("KF", "inputs", "in_1"))

    # Works with other sources as well, but then specify "source" instead of "action" as optional arg to connect(..) and disconnect(..).
    # NOTE: with the remove=False flag, we avoid removing terminal 'obs_1'
    graph.disconnect(source=("obj", "sensors", "N6"), observation="obs_1", remove=False)

    # GUI routine for making connections
    source = ("obj", "sensors", "N6")
    target = ("env/observations", "inputs", "obs_1")
    # GUI: Identify if source/target is action/observation
    observation = target[2] if target[0] == "env/observations" else None
    action = source[2] if source[0] == "env/actions" else None
    params = graph.get_parameters(
        name=target[0], component=target[1], cname=target[2]
    )  # Grab already defined parameters from input component
    if len(params) == 0:  # If observation, dict will be empty.
        converter = graph.get_parameter(
            parameter="space_converter",
            name=source[0],
            component=source[1],
            cname=source[2],
            default=identity,
        )  # grab space_converter from source ('obj', 'sensors', 'N6')
        delay, window = 0, 0
    else:  # If not observation, these values will always be present
        converter, delay, window = (
            params["converter"],
            params["delay"],
            params["window"],
        )
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
    graph.connect(source=("N1", "outputs", "out_1"), observation="obs_6")
    graph.connect(action="act_6", target=("N1", "inputs", "in_1"), skip=True)

    # Test when KF skips all inputs at t=0
    graph.remove_component("KF", "inputs", "in_1")

    # Open gui (only opens if eagerx-gui is installed)
    graph.gui()

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
    env = EagerEnv(
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
    graph.connect(action="act_1", target=("KF", "inputs", "in_2"), skip=False)
    graph.connect(action="act_1", target=("N3", "feedthroughs", "out_1"))
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

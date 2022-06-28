import eagerx

import pytest


@pytest.mark.timeout(60)
def test_graph():
    eagerx.set_log_level(eagerx.INFO)
    rate = 7

    # Get info on various specs.
    from tests.test.processors import GetIndex
    GetIndex.info()
    from tests.test.objects import Viper
    Viper.info()
    Viper.info(method="make")
    Viper.info(method=["make"])

    # Define nodes
    from tests.test.nodes import ProcessNode, KalmanNode, RealResetNode
    N0 = ProcessNode.make("N0", rate=rate, process=eagerx.ENVIRONMENT, inputs=["in_1"], outputs=["out_1"])
    N1 = ProcessNode.make("N1", rate=rate, process=eagerx.ENVIRONMENT, inputs=["in_1"], outputs=["out_1"])
    N2 = ProcessNode.make("N2", rate=rate, process=eagerx.ENVIRONMENT, inputs=["in_1"], outputs=["out_1"])
    KF = KalmanNode.make("KF", rate=rate, process=eagerx.NEW_PROCESS, inputs=["in_1", "in_2"], outputs=["out_1", "out_2"])
    N3 = RealResetNode.make("N3", rate=rate, process=eagerx.NEW_PROCESS, inputs=["in_1", "in_2"], targets=["target_1"])

    # Define object
    viper = Viper.make("obj", position=[1.0, 1.0, 1.0], actuators=["N8"], sensors=["N6"], states=["N9"])

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

    # Define graphs in different ways
    _ = eagerx.Graph.create(nodes=N0).__str__()
    _ = N0.inputs.__repr__()
    _ = N0.inputs.__str__()
    try:
        _ = N0.inputs.dummy_cname
    except AttributeError as e:
        print("Must fail! ", e)

    graph = eagerx.Graph.create(nodes=[N3, KF], objects=viper)

    # Get specs
    graph.get_spec("KF")
    graph.get_spec("N3")
    graph.get_spec("obj")

    # Rendering
    from tests.test.processors import ToUint8
    graph.render(
        source=viper.sensors.N6,
        rate=1,
        display=False,
        processor=ToUint8.make()
    )
    graph.render(
        source=viper.sensors.N6,
        rate=1,
        display=False,
        processor=ToUint8.make()
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
    graph.connect(source=N3.outputs.out_1, target=viper.actuators.N8)

    # Set & get parameters
    _ = graph.get(N3)
    graph.set(graph.get(action="act_2"), action="act_2")
    graph.set(graph.get(observation="obs_1"), observation="obs_1")
    graph.set({"window": 1}, observation="obs_1")
    _ = graph.get(observation="obs_1", parameter="processor")
    _ = graph.get(N3.config, parameter="test_arg")
    _ = graph.get(viper.sensors.N6)
    graph.set("Modified", N3.config, parameter="test_arg")
    graph.set({"test_arg": "Modified"}, N3.config)
    graph.set([1, 1, 1], viper.config, parameter="position")

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
        processor = graph.get(source, parameter="processor")
        delay, window = 0, 0
    else:  # If not observation, these values will always be present
        processor, delay, window = params["processor"], params["delay"], params["window"]
    # GUI: open dialogue box where users can modify processor, delay, window etc... Use previous params to set initial values.
    # GUI: processor, delay, window = ConnectionOptionsDialogueBox(processor, delay, window)
    target = None if observation else target  # If we have an observation, it will be the target instead in .connect(..)
    source = None if action else source  # If we have an action, it will be the source instead in .connect(..)
    # GUI: use the modified params via the dialogue box to connect.
    graph.connect(
        source=source,
        target=target,
        action=action,
        observation=observation,
        processor=processor,
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

    # Define engine
    from tests.test.engine import TestEngine
    engine = TestEngine.make(rate=20, sync=True, real_time_factor=5.5, process=eagerx.ENVIRONMENT)

    # Define backend
    # from eagerx.backends.ros1 import Ros1
    # backend = Ros1.make()
    from eagerx.backends.single_process import SingleProcess
    backend = SingleProcess.make()

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
    from eagerx.wrappers.flatten import Flatten
    env = TestEnv(name="graph", rate=rate, graph=graph, engine=engine, backend=backend)
    env = Flatten(env)

    # Get spaces
    _ = env.observation_space
    _ = env.action_space
    _ = env.state_space

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

    # Stop rendering
    env.close()

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
    print("\n[Shutdown]")


if __name__ == "__main__":
    test_graph()

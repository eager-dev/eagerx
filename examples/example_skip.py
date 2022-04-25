import eagerx

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import


def skip_run():
    # Start roscore
    roscore = eagerx.initialize("eagerx_core", anonymous=True, log_level=eagerx.log.INFO)

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
    graph.connect(source=N1.outputs.out_2, observation="maf_state", skip=True, window=1, initial_obs=[666])
    graph.connect(source=arm.sensors.N6, observation="sens_1")

    # Define bridge
    bridge = eagerx.Bridge.make("TestBridge", rate=20, sync=True, real_time_factor=0,
                                process=eagerx.process.ENVIRONMENT)

    # Initialize Environment
    env = eagerx.EagerxEnv(name="rx", rate=7, graph=graph, bridge=bridge, force_start=False)

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

    # Shutdown test
    # env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    skip_run()
    skip_run()

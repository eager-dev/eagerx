from eagerx import Object, Engine
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph

# Implementation specific
import tests.test  # noqa # pylint: disable=unused-import

import pytest


@pytest.mark.timeout(60)
def test_graph_engine():
    # Start roscore
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.WARN)

    # Define object
    arm = Object.make("Arm", "obj", actuators=["N8"], sensors=["N6"], states=["N9"])

    # Define graph
    graph = Graph.create(objects=[arm])

    # Connect sensors (= outputs of object)
    graph.connect(source=arm.sensors.N6, observation="obs_1")
    graph.connect(action="act_1", target=arm.actuators.N8)

    # Define engine
    engine = Engine.make("TestEngine", rate=20, sync=True, real_time_factor=0, process=process.ENVIRONMENT)

    # Initialize Environment
    env = EagerxEnv(name="graph_engine", rate=7, graph=graph, engine=engine)

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
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")

import rospy
from eagerx import Object, Bridge, Node, ResetNode, Converter, BaseConverter
from eagerx import initialize, log, process
roscore = initialize("eagerx_core", anonymous=True, log_level=log.DEBUG)

# Environment imports
from eagerx.core.env import EagerEnv
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.bridges.test  # noqa # pylint: disable=unused-import

import time


def graph_engine(idx):
    # Start roscore
    # roscore = initialize("eagerx_core", anonymous=True, log_level=log.DEBUG)

    # Define object
    arm = Object.make(
        "Arm",
        "obj",
        actuators=["N8"],
        sensors=["N6"],
        states=["N9"],
    )

    # Define graph
    graph = Graph.create(objects=[arm])

    # Connect sensors (= outputs of object)
    graph.connect(source=arm.sensors.N6, observation="obs_1")
    graph.connect(action="act_1", target=arm.actuators.N8)

    # Define bridge
    bridge = Bridge.make("TestBridge", rate=20, is_reactive=True, real_time_factor=1, process=process.ENVIRONMENT)

    # Initialize Environment
    env = EagerEnv(name=f"rx_{idx}", rate=7, graph=graph, bridge=bridge)

    # First reset
    env.reset()
    # env.render(mode="human")
    # action = env.action_space.sample()
    # for j in range(2):
    #     print("\n[Episode %s]" % j)
    #     for i in range(50):
    #         env.step(action)
    #         env.render(mode="rgb_array")
    #     env.reset()
    # print("\n[Finished]")

    # Shutdown test

    env.shutdown()
    if roscore:
        roscore.shutdown()
    # rospy.sleep(20.0)
    print("\n[Shutdown]")


if __name__ == "__main__":
    # Start roscore
    # roscore = initialize("eagerx_core", anonymous=True, log_level=log.DEBUG)
    for idx in range(1):
        graph_engine(str(time.time()).replace('.', '_'))
    # if roscore:
    #     roscore.shutdown()

from eagerx import Object, Bridge, Node, Processor, SpaceConverter
from eagerx import initialize, log, process

# Environment imports
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.bridges.openai_gym as eagerx_gym
import eagerx.nodes  # noqa: F401
import eagerx.converters  # noqa: F401

import time

zero_action = {"Pendulum-v0": [0.0], "Acrobot-v1": 0}


def graph_engine(idx):
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.DEBUG)

    gym_id = "Pendulum-v0"
    is_reactive = True
    rtf = 0
    p = 0

    # Define rate (depends on rate of gym env)
    rate = 20
    name = gym_id.split("-")[0]
    za = zero_action[gym_id]

    # Get signature of object
    Object.get_spec("GymObject")

    # Create object
    obj = Object.make("GymObject", name, env_id=gym_id, rate=rate, default_action=za)

    # Define graph
    graph = Graph.create(objects=[obj])

    # Add butterworth filter
    get_index = Processor.make("GetIndex_Float32MultiArray", index=0)
    bf = Node.make("ButterworthFilter", name="bf", rate=rate, N=2, Wn=4, process=process.ENVIRONMENT)
    graph.add(bf)
    graph.connect(source=obj.sensors.observation, target=bf.inputs.signal, converter=get_index)
    sc = SpaceConverter.make("Space_Float32MultiArray", [-3], [3], dtype="float32")
    graph.connect(source=bf.outputs.filtered, observation="filtered", converter=sc)

    # Connect gym object
    graph.connect(source=obj.sensors.observation, observation="observation", window=1)
    graph.connect(source=obj.sensors.reward, observation="reward", window=1)
    graph.connect(source=obj.sensors.done, observation="done", window=1)
    graph.connect(action="action", target=obj.actuators.action, window=1)

    # Define bridge
    bridge = Bridge.make("GymBridge", rate=rate, is_reactive=is_reactive, real_time_factor=rtf, process=p)

    # Initialize Environment
    name = str(time.time()).replace('.', '_')
    env = eagerx_gym.EagerGym(name=f"rx_{name}", rate=rate, graph=graph, bridge=bridge)

    # First reset
    env.reset()

    # Shutdown
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")


if __name__ == "__main__":
    graph_engine(str(time.time()).replace('.', '_'))


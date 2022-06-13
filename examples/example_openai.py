# EAGERx imports
import eagerx
from eagerx import Object, Engine, initialize, log, process

initialize("eagerx_core", anonymous=True, log_level=log.INFO)

# Environment imports
from eagerx.core.graph import Graph


# Implementation specific
import eagerx.engines.openai_gym as eagerx_gym


if __name__ == "__main__":
    # todo: make sure all tests pass again.
    # todo: refactor tests.test.node.py
    # Define rate (depends on rate of gym env)
    rate = 20

    # Define object
    gym_id = "Acrobot-v1"  # 'Pendulum-v0', 'Acrobot-v1', 'CartPole-v1', 'MountainCarContinuous-v0'
    name = gym_id.split("-")[0]
    obj = Object.make("GymObject", name, env_id=gym_id, rate=rate, default_action=0, render_shape=[300, 300])

    # Define graph
    graph = Graph.create(objects=[obj])
    graph.connect(source=obj.sensors.observation,   observation="observation",      window=1)
    graph.connect(source=obj.sensors.reward,        observation="reward",           window=1)
    graph.connect(source=obj.sensors.done,          observation="done",             window=1)
    graph.connect(action="action",                  target=obj.actuators.action,    window=1)

    # Add rendering
    graph.add_component(obj.sensors.image)
    graph.render(source=obj.sensors.image, rate=10)

    # Open gui
    # graph.gui()

    # Test save & load functionality
    graph.save("./test.graph")
    graph.load("./test.graph")

    # Define engine
    engine = Engine.make("GymEngine", rate=rate, sync=True, real_time_factor=1, process=process.NEW_PROCESS)

    env = eagerx_gym.EagerxGym(name="rx", rate=rate, graph=graph, engine=engine)
    env.render(mode="human")
    done, obs = False, env.reset()
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
    env.shutdown()

    # Initialize Environment
    env = eagerx_gym.EagerxGym(name="rx", rate=rate, graph=graph, engine=engine)

    # Turn on rendering
    # env.render(mode="human")

    # First reset
    obs, done = env.reset(), False
    for j in range(5):
        print("\n[Episode %s]" % j)
        iter = 0
        env.render(mode="human")
        while not done and iter < 20:
            iter += 1
            if iter == 10:
                env.close()
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
        obs = env.reset()
        done = False
    print("\n[Finished]")
    env.shutdown()
    print("\n[shutdown]")


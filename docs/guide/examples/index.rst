*************
Code Examples
*************

Below you can find a code example of environment creation and training using `Stable-Baselines3 <https://stable-baselines3.readthedocs.io/en/master/>`_.
To run this code, you should install `eagerx_tutorials <https://github.com/eager-dev/eagerx_tutorials>`_, which can be done by running:

.. code:: shell

    pip3 install eagerx_tutorials

Detailed explanation of the code can be found in `this Colab tutorial <https://colab.research.google.com/github/eager-dev/eagerx_tutorials/blob/master/tutorials/pendulum/1_environment_creation.ipynb>`_.

.. code-block:: python

    import eagerx
    from eagerx.backends.single_process import SingleProcess
    from eagerx.wrappers import Flatten
    from eagerx_tutorials.pendulum.objects import Pendulum
    from eagerx_ode.engine import OdeEngine

    import stable_baselines3 as sb3
    import numpy as np
    from typing import Dict


    class PendulumEnv(eagerx.BaseEnv):
        def __init__(self, name: str, rate: float, graph: eagerx.Graph, engine: eagerx.specs.EngineSpec,
                     backend: eagerx.specs.BackendSpec):
            self.max_steps = 100
            self.steps = None
            super().__init__(name, rate, graph, engine, backend, force_start=True)

        def step(self, action: Dict):
            observation = self._step(action)
            self.steps += 1

            th = observation["angle"][0]
            thdot = observation["angular_velocity"][0]
            u = float(action["voltage"])
            th -= 2 * np.pi * np.floor((th + np.pi) / (2 * np.pi))

            cost = th ** 2 + 0.1 * thdot ** 2 + 0.01 * u ** 2
            done = self.steps > self.max_steps
            info = {"TimeLimit.truncated": self.steps > self.max_steps}
            return observation, -cost, done, info

        def reset(self) -> Dict:
            states = self.state_space.sample()
            observation = self._reset(states)
            self.steps = 0
            return observation

    if __name__ == "__main__":
        rate = 30.0

        pendulum = Pendulum.make("pendulum", actuators=["u"], sensors=["theta", "theta_dot"], states=["model_state"])

        graph = eagerx.Graph.create()
        graph.add(pendulum)
        graph.connect(action="voltage", target=pendulum.actuators.u)
        graph.connect(source=pendulum.sensors.theta, observation="angle")
        graph.connect(source=pendulum.sensors.theta_dot, observation="angular_velocity")

        engine = OdeEngine.make(rate=rate)
        backend = SingleProcess.make()

        env = PendulumEnv(name="PendulumEnv", rate=rate, graph=graph, engine=engine, backend=backend)
        env = Flatten(env)

        model = sb3.SAC("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=int(150 * rate))

        env.shutdown()
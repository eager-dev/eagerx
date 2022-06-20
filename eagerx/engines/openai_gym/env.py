from eagerx import Graph
from eagerx.core.specs import EngineSpec, BackendSpec
from eagerx.core.env import BaseEnv
from typing import Dict, Tuple
import gym


class EagerxGym(BaseEnv):
    def __init__(
        self, name: str, rate: float, graph: Graph, engine: EngineSpec, backend: BackendSpec, force_start: bool = True
    ):
        super(EagerxGym, self).__init__(name, rate, graph, engine, backend, force_start=force_start)

    @property
    def observation_space(self) -> gym.spaces.Dict:
        space = self._observation_space
        space.pop("reward")
        space.pop("done")
        return space

    @property
    def action_space(self) -> gym.spaces.Dict:
        return self._action_space

    def reset(self):
        # Sample desired states
        states = self.state_space.sample()

        # Perform reset
        obs = self._reset(states)
        obs.pop("reward")
        obs.pop("done")
        return obs

    def step(self, action: Dict) -> Tuple[Dict, float, bool, Dict]:
        obs = self._step(action)
        return obs, obs.pop("reward"), bool(obs.pop("done")), dict()

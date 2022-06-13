from eagerx import Graph
from eagerx.core.specs import EngineSpec
from eagerx.core.env import BaseEnv
from typing import Dict, Tuple
import gym


class EagerxGym(BaseEnv):
    def __init__(self, name: str, rate: float, graph: Graph, engine: EngineSpec, force_start: bool = True):
        super(EagerxGym, self).__init__(name, rate, graph, engine, force_start=force_start)

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


# def step_fn(prev_obs, obs, action, steps):
#     info = dict()
#     return obs, obs.get("reward", 0.0)[0], obs.get("done", False)[0], info


# class EagerxGym(EagerxEnv):
#     def __init__(
#         self,
#         name: str,
#         rate: float,
#         graph: Graph,
#         engine: EngineSpec,
#         step_fn: Callable = step_fn,
#     ) -> None:
#         super().__init__(name=name, rate=rate, graph=graph, engine=engine, step_fn=step_fn, exclude=["reward", "done"])
#         # Flatten action spaces
#         self._reduced_action_space = super(EagerxGym, self).action_space
#         self._flattened_action_space, self._actions_all_discrete = get_flattened_space(self._reduced_action_space)
#
#         # Flatten & reduce observation spaces
#         self._reduced_obs_space = super(EagerxGym, self).observation_space
#         self._flattened_obs_space, self._obs_all_discrete = get_flattened_space(self._reduced_obs_space)
#         assert not self._obs_all_discrete, "Only continuous observations are currently supported."
#
#     @property
#     def observation_space(self):
#         return self._flattened_obs_space
#
#     @property
#     def action_space(self):
#         return self._flattened_action_space
#
#     def unflatten_action(self, action):
#         # Unflatten action
#         if not isinstance(action, np.ndarray):  # Discrete space
#             action = np.array([action])
#         return gym.spaces.unflatten(self._reduced_action_space, action)
#
#     def flatten_observation(self, obs):
#         if not self._obs_all_discrete:
#             return gym.spaces.flatten(self._reduced_obs_space, obs)
#         else:
#             raise ValueError("Only continuous observations are currently supported.")
#
#     def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
#         action = self.unflatten_action(action)
#
#         # Apply action
#         obs, reward, is_done, info = super(EagerxGym, self).step(action)
#
#         # Flatten observation
#         obs = self.flatten_observation(obs)
#         return obs, reward, is_done, info
#
#     def reset(self):
#         obs = super(EagerxGym, self).reset()
#         obs = self.flatten_observation(obs)
#         return obs


# def get_flattened_space(spaces):
#     if isinstance(spaces, gym.spaces.dict.Dict):
#         spaces = spaces.__dict__["spaces"]
#     if not isinstance(spaces, dict):
#         spaces = dict(spaces)
#     # Check if all discrete or mixed (with continuous)
#     all_discrete = True
#     for _key, space in spaces.items():
#         if isinstance(space, gym.spaces.Box) and not (space.dtype == "int64" and space.shape == (1,)):
#             all_discrete = False
#         elif isinstance(space, gym.spaces.MultiDiscrete):
#             raise ValueError("MultiDiscrete space not supported.")
#     # If all discrete & multiple discrete, initialize MultiDiscrete, else just discrete
#     if all_discrete and len(spaces) > 1:
#         multi = []
#         for _key, space in spaces.items():
#             multi.append(space.high[0] + 1)
#         flattened_space = gym.spaces.MultiDiscrete(multi)
#     elif all_discrete and len(spaces) == 1:
#         key = list(spaces)[0]
#         flattened_space = gym.spaces.Discrete(spaces[key].high[0] + 1)
#     else:
#         for key, space in spaces.items():
#             if isinstance(space, gym.spaces.Box) and space.dtype == "int64" and space.shape == (1,):
#                 spaces[key] = gym.spaces.Box(
#                     low=space.low.astype("float32"),
#                     high=space.high.astype("float32") + 0.9999,
#                     dtype="float32",
#                 )
#         flattened_space = gym.spaces.flatten_space(gym.spaces.Dict(spaces))
#     return flattened_space, all_discrete

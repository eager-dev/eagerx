from typing import Dict
import gym.spaces
from gym.core import ObservationWrapper


class RemoveRewardDoneObservation(ObservationWrapper):
    """
    Observation wrapper that takes the last msg as the observation.

    :param env: The environment to flatten
    """

    def __init__(self, env: gym.Env) -> None:
        super(RemoveRewardDoneObservation, self).__init__(env)

    @property
    def observation_space(self):
        observation_space = dict(self.env.observation_space)
        observation_space.pop("reward", None)
        observation_space.pop("done", None)
        return gym.spaces.Dict(spaces=observation_space)

    def observation(self, observation: Dict) -> Dict:
        observation.pop("reward", None)
        observation.pop("done", None)
        return observation

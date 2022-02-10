from typing import Dict
import gym
from gym.core import ActionWrapper, ObservationWrapper
from gym.wrappers import FlattenObservation


class TakeLastObservations(ObservationWrapper):
    """
    Observation wrapper that takes the last msg as the observation.

    :param env: The environment to flatten
    """

    def __init__(self, env: gym.Env) -> None:
        super(TakeLastObservations, self).__init__(env)

    def observation(self, observation: Dict) -> Dict:
        for cname, value in observation.items():
            observation[cname] = [value[-1]]
        return observation


class FlattenAction(ActionWrapper):
    """
    Observation wrapper that flattens the observation.

    :param env: The environment to flatten
    """

    def __init__(self, env: gym.Env) -> None:
        super(FlattenAction, self).__init__(env)
        self.action_space = gym.spaces.flatten_space(env.action_space)

    def action(self, action: object) -> object:
        return gym.spaces.unflatten(self.env.action_space, action)


class Flatten(gym.Wrapper):
    """
    Flattens the environment action and observations space to a single
    type.

    :param env: The environment to flatten
    """

    def __init__(self, env: gym.Env) -> None:
        super().__init__(FlattenObservation(FlattenAction(env)))

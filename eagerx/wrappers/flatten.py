import gym
from gym.core import ActionWrapper
from gym.wrappers import FlattenObservation


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

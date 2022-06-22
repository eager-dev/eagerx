import abc
import numpy as np
import typing


class Publisher(object):
    @abc.abstractmethod
    def publish(self, msg: typing.Union[float, bool, int, str, np.ndarray, np.number]) -> None:
        pass

    @abc.abstractmethod
    def unregister(self) -> None:
        pass


class Subscriber(object):
    @abc.abstractmethod
    def unregister(self) -> None:
        pass


class ShutdownService(object):
    @abc.abstractmethod
    def unregister(self) -> None:
        pass

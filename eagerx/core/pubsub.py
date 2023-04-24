import abc
import numpy as np
import typing

from eagerx.utils.utils import Header

if typing.TYPE_CHECKING:
    from eagerx.core.entities import Backend


class Publisher(object):
    def __init__(self, backend: "Backend"):
        self._backend = backend
        self._seq = 0

    @abc.abstractmethod
    def _publish(self, msg: typing.Union[float, bool, int, str, np.ndarray, np.number], header: Header) -> None:
        pass

    def publish(
        self, msg: typing.Union[float, bool, int, str, np.ndarray, np.number], header: typing.Optional[Header] = None
    ) -> None:
        if header is None:
            sc, wc = self._backend.now()
            header = Header(seq=self._seq, sc=sc, wc=wc)
        else:
            header._replace(seq=self._seq)
        self._publish(msg, header)

    @abc.abstractmethod
    def unregister(self) -> None:
        pass


class Subscriber(object):
    def __init__(self, backend: "Backend", header: bool):
        self._backend = backend
        self._header = header

    @abc.abstractmethod
    def unregister(self) -> None:
        pass


class ShutdownService(object):
    @abc.abstractmethod
    def unregister(self) -> None:
        pass

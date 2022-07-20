import copy
import numpy as np
import typing
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import cpu_count
from eagerx.core.pubsub import Publisher, Subscriber, ShutdownService
import eagerx
from eagerx.core.constants import (
    WARN,
    BackendException,
    Unspecified,
)


# A singleton that is used to check if an argument was specified.
_unspecified = Unspecified()


def merge(a: typing.Dict, b: typing.Dict, path=None):
    """merges b into a"""
    if path is None:
        path = []
    for key in b:
        if key in a:
            if isinstance(a[key], dict) and isinstance(b[key], dict):
                merge(a[key], b[key], path + [str(key)])
            elif a[key] == b[key]:
                pass  # same leaf value
            else:
                a[key] = b[key]
        else:
            a[key] = b[key]
    return a


def split(a: typing.Any):
    if isinstance(a, dict):
        for key in list(a):
            value = a.pop(key)
            value = split(value)
            keys = [k for k in key.split("/") if len(k) > 0]
            for kk in reversed(keys[1:]):
                value = {kk: value}
            if keys[0] not in a:
                a[keys[0]] = value
            else:
                merge(a[keys[0]], value)
    return a


class SingleProcess(eagerx.Backend):

    BACKEND = "SINGLE_PROCESS"
    DISTRIBUTED_SUPPORT = False
    MULTIPROCESSING_SUPPORT = False
    COLAB_SUPPORT = True

    MIN_THREADS = 10

    @classmethod
    def make(cls, log_level=WARN) -> eagerx.specs.BackendSpec:
        spec = cls.get_specification()
        spec.config.log_level = log_level if isinstance(log_level, str) else eagerx.get_log_level()
        return spec

    def initialize(self):
        self._bnd = self
        self._pserver = dict()
        self._topics = dict()
        self._cond = threading.Condition()
        self._tpool = ThreadPoolExecutor(max_workers=max(self.MIN_THREADS, cpu_count()))

    def Publisher(self, address: str, dtype: str):
        return _Publisher(self._bnd, self._tpool, self._topics, self._cond, address, dtype)

    def Subscriber(self, address: str, dtype: str, callback, callback_args=tuple()):
        return _Subscriber(self._bnd, self._topics, self._cond, address, dtype, callback, callback_args=callback_args)

    def register_environment(self, name: str, force_start: bool, fn: typing.Callable):
        return _ShutdownService()

    def delete_param(self, param: str, level: int = 1) -> None:
        try:
            keys = [k for k in param.split("/") if len(k) > 0]
            val = self._pserver
            for key in keys[:-1]:
                val = val[key]
            val.pop(keys[-1])
            self.loginfo(f'Parameters under namespace "{param}" deleted.')
        except KeyError as e:
            if level == 0:
                raise BackendException(e)
            elif level == 1:
                self.logwarn(e)
            else:
                pass

    def upload_params(self, ns: str, values: typing.Dict, verbose: bool = False) -> None:
        values = copy.deepcopy(values)
        ns = [k for k in ns.split("/") if len(k) > 0]
        ns_values = split(values)
        for k in ns:
            ns_values = {k: ns_values}
        merge(self._pserver, ns_values)

    def get_param(self, name: str, default: typing.Any = _unspecified):
        try:
            keys = [k for k in name.split("/") if len(k) > 0]
            val = self._pserver
            for key in keys:
                val = val[key]
            return val
        except KeyError as e:
            if not isinstance(default, Unspecified):
                return default
            else:
                raise BackendException(e)

    def spin(self):
        raise NotImplementedError(f"Not implemented, because backend '{self.BACKEND}' does not support multiprocessing.")

    def shutdown(self) -> None:
        if not self._has_shutdown:
            self.logdebug("Backend.shutdown() called.")
            self._has_shutdown = True
            self._tpool.shutdown(wait=True)


class _Publisher(Publisher):
    def __init__(
        self,
        backend: eagerx.Backend,
        tpool: ThreadPoolExecutor,
        topics: typing.Dict,
        cond: threading.Condition,
        address: str,
        dtype: str,
    ):
        self._bnd = backend
        self._tpool = tpool
        self._cond = cond
        self._topics = topics
        with self._cond:
            if address not in topics:
                self._topic = dict(pubs=0, subs=[], latched=None, dtype=dtype)
                topics[address] = self._topic
            else:
                self._topic = topics[address]
                assert self._topic["dtype"] == dtype, f"Dtypes do not match for topic {address}."

            # Increase publisher count
            self._topic["pubs"] += 1

            self._address = address
            self._dtype = dtype
            self._name = f"{self._address}"
            self._unregistered = False

    def publish(self, msg: typing.Union[float, bool, int, str, np.ndarray, np.number]) -> None:
        if not self._unregistered:
            # todo: check if dtype(msg) == self._dtype?
            # Convert python native types to numpy arrays.
            if isinstance(msg, float):
                msg = np.array(msg, dtype="float32")
            elif isinstance(msg, int) and not isinstance(msg, bool):
                msg = np.array(msg, dtype="int64")

            # Check if message complies with space
            if not isinstance(msg, (np.ndarray, np.number, str, bool)):
                self._bnd.logerr(f"[publisher][{self._name}]: type(recv)={type(msg)}")
                time.sleep(10000000)

            # with self._cond:  # todo: needed?
            for cb in self._topic["subs"]:
                self._tpool.submit(cb, msg)
            self._topic["latched"] = msg

    def unregister(self) -> None:
        if not self._unregistered:
            with self._cond:
                self._unregistered = True
                assert self._topic["pubs"] > 0, "According to the counter, there should be no publishers left for this topic."
                self._topic["pubs"] -= 1

                # If no other subscribers or publishers, remove topic.
                if len(self._topic["subs"]) == 0 and self._topic["pubs"] == 0:
                    self._topics.pop(self._address)


class _Subscriber(Subscriber):
    def __init__(
        self,
        backend: eagerx.Backend,
        topics: typing.Dict,
        cond: threading.Condition,
        address: str,
        dtype: str,
        callback,
        callback_args=tuple(),
    ):
        self._bnd = backend
        self._cond = cond
        self._topics = topics
        with self._cond:
            if address not in topics:
                self._topic = dict(pubs=0, subs=[], latched=None, dtype=dtype)
                topics[address] = self._topic
                latched = None
            else:
                self._topic = topics[address]
                assert self._topic["dtype"] == dtype, f"Dtypes do not match for topic {address}."
                latched = self._topic["latched"]

            self._unregistered = False
            self._topic["subs"].append(self.callback)
            self._address = address
            self._dtype = dtype
            self._cb_wrapped = callback
            self._cb_args = callback_args
            self._name = f"{self._address}"

        if latched is not None:
            self._bnd.logdebug(f"LATCHED: {self._address}")
            self.callback(latched)  # todo: inside cond?

    def callback(self, msg):
        if not self._unregistered:
            # todo: check if dtype(msg) == self._dtype?
            if not isinstance(msg, (np.ndarray, np.number, str, bool)):
                self._bnd.logerr(f"[subscriber][{self._name}]: type(recv)={type(msg)}")
                time.sleep(10000000)
            self._cb_wrapped(msg, *self._cb_args)

    def unregister(self) -> None:
        if not self._unregistered:
            with self._cond:
                self._unregistered = True

                self._topic["subs"] = [cb for cb in self._topic["subs"] if not id(cb) == id(self.callback)]

                # If no other subscribers or publishers, remove topic.
                if len(self._topic["subs"]) == 0 and self._topic["pubs"] == 0:
                    self._topics.pop(self._address)


class _ShutdownService(ShutdownService):
    def __init__(self):
        pass

    def unregister(self):
        pass

from typing import List, Dict, Optional, Union, Any, Callable, Tuple
import abc
import inspect
import os
import time
import numpy as np
import psutil
from tabulate import tabulate
from termcolor import colored

from eagerx.core.log import frame_to_caller_id, LoggingOnce
import eagerx
from eagerx.core.pubsub import Publisher, Subscriber, ShutdownService
from eagerx.core.specs import ObjectSpec
from eagerx.core.constants import (
    ENGINE,
    SILENT,
    DEBUG,
    INFO,
    ERROR,
    WARN,
    FATAL,
    Unspecified,
)
from eagerx.core.rx_message_broker import RxMessageBroker
from eagerx.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx.utils.utils import (
    Msg,
    load,
    initialize_processor,
    get_param_with_blocking,
)

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from eagerx.core.graph_engine import EngineGraph  # noqa: F401
    from eagerx.core.specs import (  # noqa:
        EntitySpec,
        EngineSpec,
        NodeSpec,
        ProcessorSpec,
        EngineStateSpec,
        ResetNodeSpec,
        ObjectSpec,
        BackendSpec,
    )


# A singleton that is used to check if an argument was specified.
_unspecified = Unspecified()


class Entity(object):
    @classmethod
    def get_specification(cls):
        entity_type = cls.__module__ + "/" + cls.__qualname__
        spec = cls.pre_make(entity_type, entity_type)
        spec.initialize(cls)
        return spec

    @classmethod
    @abc.abstractmethod
    def make(cls, *args: Any, **kwargs: Any):
        """An abstract method that makes the specification (also referred to as `spec`) of this entity.

        :param args: Arguments to the subclass' make function.
        :param kwargs: Optional Arguments to the subclass' make function.
        :return: A (mutable) spec that can be used to build and subsequently initialize the entity (e.g. node, engine, ...).
        """
        pass

    @classmethod
    def info(cls, method: Optional[Union[List[str], str]] = None) -> str:
        """A helper method to get info on a method of the specified subclass.

        :param method: The registered method we would like to receive info on. If no method is specified, it provides info on
                       the class itself.
        :return: Info on the subclass' method.
        """
        from eagerx.core import info

        return info.get_info(cls, method)

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        from eagerx.core.specs import EntitySpec  # noqa: F811 F401

        return EntitySpec(dict(entity_id=entity_id, entity_type=entity_type))

    @classmethod
    def check_spec(cls, spec):
        pass


class Backend(Entity):
    """Baseclass for backends.

    Use this baseclass to implement backends that implement the communication.

    Users must use :func:`~eagerx.core.entities.Backend.make` to make the registered subclass' specification.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Backend.make`

    - :func:`~eagerx.core.entities.Backend.initialize`

    - :func:`~eagerx.core.entities.Backend.Publisher`

    - :func:`~eagerx.core.entities.Backend.Subscriber`

    - :func:`~eagerx.core.entities.Backend.register_environment`

    - :func:`~eagerx.core.entities.Backend.delete_param`

    - :func:`~eagerx.core.entities.Backend.upload_params`

    - :func:`~eagerx.core.entities.Backend.get_param`

    - :func:`~eagerx.core.entities.Backend.spin`

    Subclasses must set the following static class properties:

    - :attr:`~eagerx.core.entities.Backend.BACKEND`

    - :attr:`~eagerx.core.entities.Backend.DISTRIBUTED_SUPPORT`

    - :attr:`~eagerx.core.entities.Backend.MULTIPROCESSING_SUPPORT`

    - :attr:`~eagerx.core.entities.Backend.COLAB_SUPPORT`
    """

    INFO = {
        "make": [],
        "initialize": [],
    }

    __metaclass__ = abc.ABCMeta

    def __init__(
        self,
        ns: str,
        backend_type: str,
        entity_id: str,
        log_level: int = WARN,
        **kwargs: Union[bool, int, float, str, List, Dict],
    ):
        #: Namespace of the environment. Can be set with the `name` argument to :class:`~eagerx.core.env.BaseEnv`.
        self.ns: str = ns
        #: A unique entity_id with the structure <module>/<classname>.
        self.entity_id: str = entity_id
        #: The class definition of the subclass. Follows naming convention *<module>/<BackendClassName>*.
        #: Cannot be modified.
        self.backend_type: str = backend_type
        #: Specifies the effective log level:
        #: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Backend.spec`.
        self.log_level: int = log_level

        # Record once logged messages.
        self._logging_once = LoggingOnce()

        # Backend shutdown flag
        self._has_shutdown = False

        # Call subclass' initialize method
        self.initialize(**kwargs)

    @abc.abstractmethod
    def initialize(self, spec: "BackendSpec") -> None:
        """An abstract method to initialize the backend.

        :param spec: Specification of the node/engine.
        """
        pass

    @classmethod
    def pre_make(cls, entity_id: str, entity_type: "Entity"):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        params["config"] = dict(
            log_level=eagerx.get_log_level(),
            entity_id=params.pop("entity_id"),
        )
        params["backend_type"] = params.pop("entity_type")
        from eagerx.core.specs import BackendSpec  # noqa: F811

        return BackendSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)

    @staticmethod
    def from_cmd(ns: str, backend_id: str, log_level: int, *args, **kwargs) -> "Backend":
        backend_cls = load(backend_id)
        backend = backend_cls(ns, backend_id, backend_id, log_level, *args, **kwargs)
        return backend

    @property
    @abc.abstractmethod
    def BACKEND(self) -> str:
        """Backend name in string format."""
        pass

    @property
    @abc.abstractmethod
    def DISTRIBUTED_SUPPORT(self) -> bool:
        """Whether nodes can be launched on external platforms (i.e. distributed communication)."""
        pass

    @property
    @abc.abstractmethod
    def MULTIPROCESSING_SUPPORT(self) -> bool:
        """Whether nodes can be launched as separate processes."""
        pass

    @property
    @abc.abstractmethod
    def COLAB_SUPPORT(self) -> bool:
        """Whether the backend supports running on Google colab."""
        pass

    @abc.abstractmethod
    def Publisher(self, address: str, dtype: str) -> Publisher:
        """Creates a publisher.

        :param address: Topic name.
        :param dtype: Dtype of message in string format (e.g. `float32`).
        """
        pass

    @abc.abstractmethod
    def Subscriber(self, address: str, dtype: str, callback, callback_args: Optional[Tuple] = tuple()) -> Subscriber:
        """Creates a subscriber.

        :param address: Topic name.
        :param dtype: Dtype of message in string format (e.g. `float32`).
        :param callback: Function to call ( fn(data)) when data is received. If callback_args is set, the function
                         must accept the callback_args as positional args, i.e. fn(data, *callback_args).
        :param callback_args: Additional arguments to pass to the callback.
        """
        pass

    @abc.abstractmethod
    def register_environment(self, name: str, force_start: bool, fn: Callable) -> ShutdownService:
        """Checks if environment already exists and shuts it down if `force_restart` is set. Then, it registers
        the remote shutdown procedure for the newly created environment.

        :param name: Environment name (i.e. namespace of the environment).
        :param force_start: Whether to shutdown any environment with the same name if it already exists.
        :param fn: Function with zero args to be called on remote shutdown.
        """
        pass

    @abc.abstractmethod
    def delete_param(self, param: str, level: int = 1) -> None:
        """Deletes params from the parameter server.

        :param param: Parameter name.
        :param level: Determines what to do when the param does not exist:

                      - 0=error: Raises a BackendException.

                      - 1=warn: logs a warning and returns `None`.

                      - 2=pass: passes silently and returns `None`.
        """
        pass

    @abc.abstractmethod
    def upload_params(
        self, ns: str, values: Dict[str, Union[Dict, List, bool, float, int, str]], verbose: bool = False
    ) -> None:
        """Upload params to the parameter server.

        :param ns: Namespace to load parameters into, ``str``.
        :param values: Key/value dictionary, where keys are parameter names and values are parameter values, ``dict``.
        :param verbose: Verbosity level.
        """
        pass

    @abc.abstractmethod
    def get_param(self, name: str, default: Any = _unspecified) -> Union[Dict, List, bool, float, int, str]:
        """Retrieve a parameter from the param server

        :param name: Parameter name.
        :param default: Default value to return.
        """
        pass

    @abc.abstractmethod
    def spin(self) -> None:
        """Blocks until node is shutdown. Yields activity to other threads."""
        pass

    def shutdown(self) -> None:
        """Shuts down the backend"""
        if not self._has_shutdown:
            self._has_shutdown = True
            self.logdebug("Backend.shutdown() called.")

    def logdebug_once(self, msg, identifier=None) -> None:
        caller_id = frame_to_caller_id(inspect.currentframe().f_back.f_back)
        if isinstance(identifier, str):
            caller_id += str.encode(identifier)
        if self._logging_once(caller_id):
            return self.logdebug(msg)

    def loginfo_once(self, msg, identifier=None) -> None:
        caller_id = frame_to_caller_id(inspect.currentframe().f_back.f_back)
        if isinstance(identifier, str):
            caller_id += str.encode(identifier)
        if self._logging_once(caller_id):
            return self.loginfo(msg)

    def logwarn_once(self, msg, identifier=None) -> None:
        caller_id = frame_to_caller_id(inspect.currentframe().f_back)
        if isinstance(identifier, str):
            caller_id += str.encode(identifier)
        if self._logging_once(caller_id):
            return self.logwarn(msg)

    def logerr_once(self, msg, identifier=None) -> None:
        caller_id = frame_to_caller_id(inspect.currentframe().f_back.f_back)
        if isinstance(identifier, str):
            caller_id += str.encode(identifier)
        if self._logging_once(caller_id):
            return self.logerr(msg)

    def logfatal_once(self, msg, identifier=None) -> None:
        caller_id = frame_to_caller_id(inspect.currentframe().f_back.f_back)
        if isinstance(identifier, str):
            caller_id += str.encode(identifier)
        if self._logging_once(caller_id):
            return self.logfatal(msg)

    def logdebug(self, msg) -> None:
        return self._log(f"[DEBUG]: {msg}", DEBUG, None)

    def loginfo(self, msg) -> None:
        return self._log(f"[INFO]: {msg}", INFO, None)

    def logwarn(self, msg) -> None:
        return self._log(f"[WARN]: {msg}", WARN, "red")

    def logerr(self, msg) -> None:
        return self._log(f"[ERROR]: {msg}", ERROR, "red")

    def logfatal(self, msg: str) -> None:
        return self._log(f"[FATAL]: {msg}", FATAL, "red")

    def _log(self, msg: str, level: int, color: Union[str, None]) -> None:
        if level >= self.log_level:
            print(colored(msg, color))

    def get_log_fn(self, log_level: int) -> Callable:
        if log_level == DEBUG:
            return self.logdebug
        elif log_level == INFO:
            return self.loginfo
        elif log_level == WARN:
            return self.logwarn
        elif log_level == ERROR:
            return self.logerr
        elif log_level == FATAL:
            return self.logfatal
        else:

            def logsilent(msg, *args, **kwargs):
                pass

            return logsilent


class BaseNode(Entity):
    def __init__(
        self,
        ns: str,
        message_broker: RxMessageBroker,
        sync: bool,
        real_time_factor: float,
        simulate_delays: bool,
        params: Dict,
    ):
        """
        The base class from which all nodes and engines inherit.
        """
        #: Namespace of the environment. Can be set with the `name` argument to :class:`~eagerx.core.env.BaseEnv`.
        self.ns: str = ns
        #: User specified node name. Can be set in :func:`~eagerx.core.entities.Node.spec`.
        self.name: str = params["config"]["name"]
        #: User specified node name with the namespace prepended. Cannot be modified directly.
        self.ns_name: str = f"{ns}/{self.name}"
        #: Responsible for all I/O communication within this process.
        #: Nodes inside the same process share the same backend. Cannot be modified.
        self.message_broker: RxMessageBroker = message_broker
        #: Responsible for all I/O communication within this process.
        #: Nodes inside the same process share the same message broker. Cannot be modified.
        self.backend: Backend = message_broker.bnd
        #: A unique entity_id with the structure <module>/<classname>.
        self.entity_id: str = params["config"]["entity_id"]
        #: Rate (Hz) at which the callback is called.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.rate: float = params["config"]["rate"]
        #: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.process: int = params["config"]["process"]
        #: Executable script that is used to launch the node (if launched in a separate process).
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.executable: str = params["config"]["executable"]
        #: Parameters for all selected inputs.
        self.inputs: dict = params["inputs"]
        #: Parameters for all selected outputs.
        self.outputs: dict = params["outputs"]
        #: Parameters for all selected states.
        self.states: dict = params["states"]
        #: Parameters for all feedthroughs corresponding to the selected outputs.
        self.feedthroughs: dict = params["feedthroughs"]
        #: Parameters for all selected targets.
        self.targets: dict = params["targets"]
        #: Flag that specifies whether we run reactive or asynchronous.
        #: Can be set in the engine's :func:`~eagerx.core.entities.Engine.spec`.
        self.sync: bool = sync
        #: A specified upper bound on the real_time factor. Wall-clock rate=real_time_factor*rate.
        #: If real_time_factor < 1 the simulation is slower than real time.
        #: Can be set in the engine's :func:`~eagerx.core.entities.Engine.spec`.
        self.real_time_factor: float = real_time_factor
        #: Flag that specifies whether input delays are simulated.
        #: You probably want to set this to False when running in the real-world.
        #: Can be set in the engine's :func:`~eagerx.core.entities.Engine.spec`.
        self.simulate_delays: bool = simulate_delays
        #: Specifies the color of logged messages & node color in the GUI.
        #: Check-out the termcolor documentation for the supported colors.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.color: str = params["config"]["color"]
        #: Specifies the log level for this node:
        #: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.log_level: int = params["config"]["log_level"]
        effective_log_level = eagerx.get_log_level()
        #: Specifies the log level for logging memory usage over time for this node:
        #: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}.
        #: Note that `log_level` has precedent over the memory level set here.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.log_memory: int = self.log_level >= effective_log_level and SILENT >= effective_log_level

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        params["node_type"] = params.pop("entity_type")
        params["config"] = dict(
            name=None,
            rate=None,
            process=0,
            inputs=[],
            outputs=[],
            states=[],
            color="grey",
            log_level=WARN,
            log_level_memory=SILENT,
            executable=None,
            entity_id=params.pop("entity_id"),
        )
        params.update(dict(inputs=dict(), outputs=dict(), states=dict()))
        from eagerx.core.specs import BaseNodeSpec

        return BaseNodeSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        entity_id = spec.config.entity_id
        name = spec.config.name
        assert name is not None and isinstance(
            name, str
        ), f'A node with entity_id "{entity_id}" has an invalid name {name}. Please provide a unique name of type string.'

        # Check that all selected cnames have a corresponding implementation
        for component in ["inputs", "outputs", "states"]:
            for cname in spec.config[component]:
                c = getattr(spec, component)
                msg = (
                    f'"{cname}" was selected as {component[:-1]}'
                    f'but it is not a registered {component[:-1]} of node "{name}". Check the spec of "{entity_id}".'
                )
                assert cname in c, msg

        # Check that components have a space
        for component in ["states"]:
            for cname, params in getattr(spec, component).items():
                msg = (
                    f'"{cname}" was defined as {component[:-1]} in "{name}", but its space ({type(params.space)}) '
                    f'is not a valid space. Check the spec of "{entity_id}".'
                )
                assert params.space is not None, msg

    @abc.abstractmethod
    def initialize(self, spec: Union["NodeSpec", "EngineSpec", "ResetNodeSpec"]) -> None:
        """An abstract method that initializes the node at run-time.

        :param spec: Specification of the node/engine.
        """
        pass

    def shutdown(self) -> None:
        """A method that can be overwritten to cleanly shutdown (e.g. release resources)."""
        pass


class Node(BaseNode):
    """Baseclass for nodes.

    Use this baseclass to implement nodes that will be added to the (agnostic) :class:`~eagerx.core.graph.Graph`.

    Users must call :func:`~eagerx.core.entities.Node.make` to make the node subclass' specification.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Node.make`

    - :func:`~eagerx.core.entities.Node.initialize`

    - :func:`~eagerx.core.entities.Node.reset`

    - :func:`~eagerx.core.entities.Node.callback`

    - :func:`~eagerx.core.entities.Node.shutdown` (optional)

    Use baseclass :class:`~eagerx.core.entities.EngineNode` instead, for nodes that will be added to
    :class:`~eagerx.core.graph_engine.EngineGraph` when specifying an engine implementation for an :class:`~eagerx.core.entities.Object`.

    Use baseclass :class:`~eagerx.core.entities.ResetNode` instead, for reset routines.
    """

    INFO = {
        "make": [],
        "initialize": [],
        "reset": ["states"],
        "callback": ["inputs", "outputs"],
    }

    def __init__(
        self,
        ns: str,
        message_broker: RxMessageBroker,
        sync: bool,
        real_time_factor: float,
        simulate_delays: bool,
        params: Dict,
        object_name: Optional[str] = None,  # Required when launching in a separate process.
        call_init: bool = True,
    ):

        # Message counter
        self.num_ticks = 0

        # Track memory usage  & speed
        self.total_ticks = 0
        self.py = psutil.Process(os.getpid())
        self.pid = os.getpid()
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 200
        self.history = []
        self.headers = [
            "pid",
            "node",
            "ticks",
            "rss",
            "diff",
            "t0",
            "vms",
            "diff",
            "t0",
            "iter_time",
            "diff",
            "t0",
        ]

        # Call baseclass constructor (which eventually calls .initialize())
        super().__init__(
            ns=ns,
            message_broker=message_broker,
            sync=sync,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            params=params,
        )

        if call_init:
            from eagerx.core.specs import NodeSpec

            self.initialize(NodeSpec(params))

        # Determine all inputs with window > 0 (and skip)
        self.skipped_cbs = 0
        self.windowed = dict()
        for cname, i in self.inputs.items():
            window = i["window"]
            skip = i["skip"]
            if window > 0:
                self.windowed[cname] = skip

        # If we run async *and* no msg for input with window > 0, then send None outputs.
        self.empty_outputs = dict()
        for cname, _ in self.outputs.items():
            self.empty_outputs[cname] = None
        for cname, _ in self.targets.items():
            self.empty_outputs[cname + "/done"] = False

    def reset_cb(self, **kwargs):
        self.num_ticks = 0
        self.skipped_cbs = 0
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg.info.done:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg.msgs[0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.reset(**kwargs)

    def callback_cb(self, node_tick: int, t_n: float, **kwargs):
        self.iter_ticks += 1
        if self.log_memory and self.iter_ticks % self.print_iter == 0:
            if self.iter_start:
                # Time steps
                iter_stop = time.time()
                if self.iter_ticks > 0:
                    iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                else:
                    iter_time = float(iter_stop - self.iter_start)

                # Memory usage request
                mem_use = (np.array(self.py.memory_info()[0:2]) / 2.0**30) * 1000  # memory use in MB...I think

                # Print info
                self.total_ticks += self.iter_ticks
                self.iter_ticks = 0

                # Log history
                if len(self.history) > 0:
                    delta_mem_rss = round(mem_use[0] - self.history[-1][3], 2)
                    delta_mem_vms = round(mem_use[1] - self.history[-1][6], 2)
                    delta_iter_time = round(iter_time - self.history[-1][9], 5)
                    cum_mem_rss = round(mem_use[0] - self.history[0][3], 2)
                    cum_mem_vms = round(mem_use[1] - self.history[0][6], 2)
                    cum_iter_time = round(iter_time - self.history[0][9], 5)
                    self.history.append(
                        [
                            self.pid,
                            self.name,
                            self.total_ticks,
                            round(mem_use[0], 1),
                            delta_mem_rss,
                            cum_mem_rss,
                            round(mem_use[1], 1),
                            delta_mem_vms,
                            cum_mem_vms,
                            iter_time,
                            delta_iter_time,
                            cum_iter_time,
                        ]
                    )
                else:
                    self.history.append(
                        [
                            self.pid,
                            self.name,
                            self.total_ticks,
                            round(mem_use[0], 1),
                            0,
                            0,
                            round(mem_use[1], 1),
                            0,
                            0,
                            iter_time,
                            0,
                            0,
                        ]
                    )
                self.backend.loginfo("\n" + tabulate(self.history, headers=self.headers))
            self.iter_start = time.time()

        # Skip callback if not all inputs with window > 0 have received at least one input.
        if (
            not self.sync or not self.num_ticks > 1
        ):  # Larger than 1 here, because we might have already ticked, but not yet received a skipped input.
            for cname, skip in self.windowed.items():
                if len(kwargs[cname].msgs) == 0:
                    if skip and self.num_ticks == 0:
                        continue
                    else:
                        self.skipped_cbs += 1
                        self.backend.logdebug(f"[{self.name}][{cname}]: skipped_cbs={self.skipped_cbs}")
                        return self.empty_outputs
        output = self.callback(t_n, **kwargs)
        self.num_ticks += 1
        # We skip output type checks for node = "environment":
        #  - The callback sometimes returns None, because .reset() was called instead of a .step().
        if self.name != "environment":
            for cname, _ in self.outputs.items():
                if cname not in output:
                    self.backend.logwarn_once(
                        f"The .callback(...) of `{self.name}` did not return the registered output `{cname}`. "
                        "Downstream nodes, depending on this output for their callback, may deadlock. "
                    )

            for cname, _ in self.targets.items():
                cname_done = cname + "/done"
                assert cname_done in output, (
                    f"The .callback(...) of `{self.name}` did not return an output dict with key"
                    f"`{cname_done}` that communicates the reset status for the registered target."
                )
                assert isinstance(output[cname_done], (bool, np.bool, np.bool_)), (
                    f"The message type of target {cname_done}` ({type(output[cname_done])}), returned by the .callback(...) of "
                    f"`{self.name}`, should be of type `bool`."
                )
        return output

    def set_delay(self, delay: float, component: str, cname: str) -> None:
        """A method to vary the delay of an input or feedthrough.

        :param delay: A non-negative delay that can be varied at the beginning of an episode (during the reset procedure).
        :param component: Either "inputs" or "feedthroughs".
        :param cname: name of the component.
        """
        assert delay >= 0, "Delay must be non-negative."
        assert cname in getattr(self, component), f"Cannot set the delay. '{cname}' is not one of the {component}."
        getattr(self, component)[cname]["delay"] = delay

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the node to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are sent by the environment and can be used to reset the node at the start of an episode.
                       This can be anything from an estimator's initial state to a hyper-parameter (e.g. delay, control gains).
        """
        pass

    @abc.abstractmethod
    def callback(self, t_n: float, **inputs: Msg) -> Dict[str, Any]:
        """An abstract method that is called at the specified node rate.

        This method should be decorated with:

        - :func:`eagerx.core.register.inputs` to register the inputs.

        - :func:`eagerx.core.register.outputs` to register the outputs.

        :param t_n: Time passed (seconds) since last reset. Increments with 1/:attr:`~eagerx.core.entities.Node.rate`.
        :param inputs: Inputs that were registered (& selected) with the :func:`eagerx.core.register.inputs` decorator by the subclass.
        :return: Dictionary with outputs that were registered (& selected) with the :func:`eagerx.core.register.outputs` decorator by the subclass.
        """
        pass

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        with spec.config as d:
            d.executable = "python:=eagerx.core.executable_node"
        from eagerx.core.specs import NodeSpec  # noqa: F811

        return NodeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        entity_id = spec.config.entity_id
        name = spec.config.name

        # Check that there is at least a single input & output defined.
        assert (
            len(spec.config.inputs) > 0 or name == "environment"
        ), f'Node "{name}" does not have any inputs selected. Please select at least one input when making the spec, or check the spec defined for "{entity_id}".'


class ResetNode(Node):
    """Baseclass for nodes that perform a reset routine.

    Use this baseclass to implement reset nodes that will be added to the (agnostic) :class:`~eagerx.core.graph.Graph`.

    Users must call :func:`~eagerx.core.entities.ResetNode.make` to make the reset node subclass' specification.

    .. note:: Subclasses must always have at least one target registered with the :func:`eagerx.core.register.targets` decorator.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.ResetNode.make`

    - :func:`~eagerx.core.entities.ResetNode.initialize`

    - :func:`~eagerx.core.entities.ResetNode.reset`

    - :func:`~eagerx.core.entities.ResetNode.callback`

    - :func:`~eagerx.core.entities.ResetNode.shutdown` (optional)

    Use baseclass :class:`~eagerx.core.entities.EngineNode` instead, for nodes that will be added to
    :class:`~eagerx.core.graph_engine.EngineGraph` when specifying an engine implementation for an :class:`~eagerx.core.entities.Object`.

    Use baseclass :class:`~eagerx.core.entities.Node` instead, for regular nodes that will be added to the
    agnostic :class:`~eagerx.core.graph.Graph`.
    """

    INFO = {
        "make": [],
        "initialize": [],
        "reset": ["states"],
        "callback": ["inputs", "outputs", "targets"],
    }

    def __init__(self, params: Dict, *args, **kwargs):
        super().__init__(*args, params=params, call_init=False, **kwargs)

        # Call initialize with spec
        from eagerx.core.specs import ResetNodeSpec

        self.initialize(ResetNodeSpec(params))

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the node to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are sent by the environment and can be used to reset the node at the start of an episode.
                       This can be anything from an estimator's initial state to a hyper-parameter (e.g. delay, control gains).
        """
        pass

    @abc.abstractmethod
    def callback(self, t_n: float, **inputs_and_targets: Msg) -> Dict[str, Any]:
        """An abstract method that is called at the specified node rate during the environment reset.

        This method should be decorated with:

        - :func:`eagerx.core.register.inputs` to register the inputs.

        - :func:`eagerx.core.register.outputs` to register the outputs.

        - :func:`eagerx.core.register.targets` to register the targets.

        .. note:: This callback is skipped until the user calls :func:`~eagerx.core.env.BaseEnv.reset`.
                  Until then, the messages coming in via the connected *feedthroughs* are fed through as
                  the outputs instead. For every registered output that was registered (& selected) with the
                  :func:`eagerx.core.register.outputs` decorator by the subclass, there must be a connected *feedthrough*.

        :param t_n: Time passed (seconds) since last reset. Increments with 1/:attr:`~eagerx.core.entities.ResetNode.rate`.
        :param inputs_and_targets: Inputs and targets that were registered (& selected) with the :func:`eagerx.core.register.inputs`
                                   and :func:`eagerx.core.register.targets` decorators by the subclass.
        :return: Dictionary with outputs that were registered (& selected) with the :func:`eagerx.core.register.outputs`
                 decorator by the subclass. In addition, the dictionary must contain message of type bool
                 that specifies whether the requested *target* was reached.
        """
        pass

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        spec._params["targets"] = dict()
        spec._params["feedthroughs"] = dict()
        with spec.config as d:
            d.targets = []

        from eagerx.core.specs import ResetNodeSpec  # noqa: F811

        return ResetNodeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        entity_id = spec.config.entity_id
        name = spec.config.name

        # Check that there is at least a single target & output was defined.
        assert (
            len(spec.config.outputs) > 0
        ), f'Node "{name}" does not have any outputs selected. Please select at least one output when making the spec, or check the spec defined for "{entity_id}".'
        assert (
            len(spec.config.targets) > 0
        ), f'Node "{name}" does not have any targets selected. Please select at least one target when making the spec, or check the spec defined for "{entity_id}".'

        # Check if all selected targets have an implementation (other components are checked in BaseNode.check_spec())
        # Check that all selected cnames have a corresponding implementation
        for component in ["targets"]:
            for cname in spec.config[component]:
                c = getattr(spec, component)
                msg = (
                    f'"{cname}" was selected as {component[:-1]}'
                    f'but it is not a registered {component[:-1]} of node "{name}". Check the spec of "{entity_id}".'
                )
                assert cname in c, msg


class EngineNode(Node):
    """Baseclass for nodes that are only to be used in combination with a specific engine.

    Users must call :func:`~eagerx.core.entities.EngineNode.make` to make the engine node subclass' specification.

    Use this baseclass to implement nodes that will be added to an :class:`~eagerx.core.graph_engine.EngineGraph`
    when specifying an engine implementation for an :class:`~eagerx.core.entities.Object`.

    These nodes can, optionally, be synchronized with respect to the simulator clock by registering "tick" as an input.

    .. note:: Engine nodes *only* receive a reference to the :attr:`~eagerx.core.entities.EngineNode.simulator` as an
              argument to :func:`~eagerx.core.entities.EngineNode.initialize` when the engine nodes are launched within
              the same process as the engine. See :class:`~eagerx.core.constants.process` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.EngineNode.make`

    - :func:`~eagerx.core.entities.EngineNode.initialize`

    - :func:`~eagerx.core.entities.EngineNode.reset`

    - :func:`~eagerx.core.entities.EngineNode.callback`

    - :func:`~eagerx.core.entities.EngineNode.shutdown` (optional)

    Use baseclass :class:`~eagerx.core.entities.Node` instead, for nodes that will be added to the
    (agnostic) :class:`~eagerx.core.graph.Graph`.

    Use baseclass :class:`~eagerx.core.entities.ResetNode` instead, for reset routines.
    """

    def __init__(
        self,
        params: Dict,
        *args,
        object_name: str = None,
        simulator: Any = None,
        message_broker: RxMessageBroker = None,
        **kwargs: Any,
    ):
        obj_params = get_param_with_blocking(object_name, message_broker.bnd)
        obj_params["engine"]["states"] = {s["name"]: s for s in obj_params["engine"]["states"]}

        # Call baseclass constructor
        super().__init__(*args, message_broker=message_broker, params=params, call_init=False, **kwargs)

        # Call initialize with spec
        from eagerx.core.specs import NodeSpec, ObjectSpec

        self.initialize(spec=NodeSpec(params), object_spec=ObjectSpec(obj_params), simulator=simulator)

    @abc.abstractmethod
    def initialize(self, spec: "NodeSpec", object_spec: "ObjectSpec", simulator: Any) -> None:
        """An abstract method that initializes the node at run-time.

        :param spec: Specification of the engine node.
        :param object_spec: Specification of the object that uses this engine node in its engine-specific implementation.
        :param simulator: A reference to the :attr:`~eagerx.core.entities.Engine.simulator`. The simulator type depends
                          on the engine. Only available if the node was launched inside the engine process.
        """
        pass

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the node to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        .. warning:: Avoid defining states for engine nodes, as you risk making your :class:`~eagerx.core.entities.Object`
                     non-agnostic to the environment. Instead, try to implement object states as an :class:`~eagerx.core.entities.EngineState`
                     of an :class:`~eagerx.core.entities.Object`.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are sent by the environment and can be used to reset the node at the start of an episode.
                       This can be anything from an estimator's initial state to a hyper-parameter (e.g. delay, control gains).
        """
        pass

    @abc.abstractmethod
    def callback(self, t_n: float, **inputs: Msg) -> Dict[str, Any]:
        """An abstract method that is called at the specified node rate.

        This method should be decorated with:

        - :func:`eagerx.core.register.inputs` to register the inputs.

        - :func:`eagerx.core.register.outputs` to register the outputs.

        :param t_n: Time passed (seconds) since last reset. Increments with 1/:attr:`~eagerx.core.entities.EngineNode.rate`.
        :param inputs: Inputs that were registered (& selected) with the :func:`eagerx.core.register.inputs` decorator by the subclass.
        :return: Dictionary with outputs that were registered (& selected) with the :func:`eagerx.core.register.outputs` decorator by the subclass.
        """
        pass

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        from eagerx.core.specs import NodeSpec  # noqa: F811

        return NodeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class Engine(BaseNode):
    """Baseclass for engines.

    Use this baseclass to implement an engine that interfaces the simulator.

    Users must call :func:`~eagerx.core.entities.Engine.make` to make the engine subclass' specification.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Engine.make`

    - :func:`~eagerx.core.entities.Engine.initialize`

    - :func:`~eagerx.core.entities.Engine.add_object`

    - :func:`~eagerx.core.entities.Engine.pre_reset`

    - :func:`~eagerx.core.entities.Engine.reset`

    - :func:`~eagerx.core.entities.Engine.callback`

    - :func:`~eagerx.core.entities.Engine.shutdown` (optional)
    """

    INFO = {
        "make": [],
        "initialize": [],
        "add_object": ["engine_config"],
        "pre_reset": ["states"],
        "reset": ["states"],
        "callback": ["outputs"],
    }

    def __init__(self, sync, real_time_factor, params, target_addresses, node_names, *args, **kwargs):
        """
        Base class constructor.

        :param sync: Boolean flag. Specifies whether we run reactive or asynchronous.
        :param real_time_factor: Sets an upper bound of real_time factor. Wall-clock rate=real_time_factor*rate.
                                 If real_time_factor < 1 the simulation is slower than real time.
        :param target_addresses: Addresses from which the engine should expect "done" msgs.
        :param simulator: Simulator object
        :param node_names: List of node names that are registered in the graph.
        :param kwargs: Arguments that are to be passed down to the node baseclass. See NodeBase for this.
        """
        #: The simulator object. The simulator depends on the engine and should be initialized in the
        #: :func:`~eagerx.core.entities.Engine.initialize` method. Oftentimes, engine nodes require a reference in
        #: :func:`~eagerx.core.entities.EngineNode.callback` and/or :func:`~eagerx.core.entities.EngineNode.reset`
        #: to this simulator object to simulate (e.g. apply an action, extract a sensor measurement).
        #: Engine nodes only have this reference if the node was launched inside the engine process.
        #: See :class:`~eagerx.core.constants.process` for more info.
        self.simulator: Any = None
        self.target_addresses = target_addresses
        self.node_names = node_names

        # Check real_time_factor & reactive args
        assert sync or (
            not sync and real_time_factor > 0
        ), "Cannot have a real_time_factor=0 while not reactive. Will result in synchronization issues. Set sync=True or real_time_factor > 0"

        # Initialized nodes
        self.sp_nodes = dict()
        self.launch_nodes = dict()
        self.num_resets = 0
        self.is_initialized = dict()

        # Message counter
        self.dtype_tick = params["outputs"]["tick"]["dtype"]
        self.num_ticks = 0

        # Track memory usage  & speed
        self.total_ticks = 0
        self.py = psutil.Process(os.getpid())
        self.pid = os.getpid()
        self.iter_start = None
        self.iter_ticks = 0
        self.print_iter = 200
        self.history = []
        self.headers = [
            "pid",
            "node",
            "ticks",
            "rss",
            "diff",
            "t0",
            "vms",
            "diff",
            "t0",
            "iter_time",
            "diff",
            "t0",
        ]

        # Call baseclass constructor (which eventually calls .initialize())
        super().__init__(*args, sync=sync, real_time_factor=real_time_factor, params=params, **kwargs)

        # Call initialize with spec
        from eagerx.core.specs import EngineSpec

        self.initialize(EngineSpec(params))

    def register_node(self, node_params):
        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        initialize_nodes(
            node_params,
            ENGINE,
            self.ns,
            self.message_broker,
            self.is_initialized,
            sp_nodes,
            launch_nodes,
        )
        wait_for_node_initialization(self.is_initialized, self.backend)
        self.sp_nodes.update(sp_nodes)
        self.launch_nodes.update(launch_nodes)
        return node_params, sp_nodes, launch_nodes

    def register_object(self, object_params, node_params, state_params):
        # Use obj_params to initialize object in simulator --> object info parameter dict is optionally added to simulation nodes
        engine_params = {key: value for key, value in object_params["engine"].items() if key != "states"}

        self.add_object(ObjectSpec(object_params), **engine_params)

        # Initialize states
        for i in state_params:
            state_cls = load(i["state"]["state_type"])
            i["state"] = state_cls(
                ns=self.ns,
                name=i["name"],
                simulator=self.simulator,
                backend=self.backend,
                params=i["state"],
                object_params=object_params,
            )
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        node_args = dict(
            object_name=f"{self.ns}/{object_params['config']['name']}",
            simulator=self.simulator,
        )
        initialize_nodes(
            node_params,
            ENGINE,
            self.ns,
            self.message_broker,
            self.is_initialized,
            sp_nodes,
            launch_nodes,
            node_args=node_args,
        )
        for _name, node in sp_nodes.items():
            # Initialize
            node.node_initialized()
        self.sp_nodes.update(sp_nodes)
        self.launch_nodes.update(launch_nodes)
        wait_for_node_initialization(self.is_initialized, self.backend)
        return state_params, sp_nodes, launch_nodes

    def pre_reset_cb(self, **kwargs):
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg.info.done:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg.msgs[0]
        [kwargs.pop(key) for key in keys_to_pop]
        return self.pre_reset(**kwargs)

    def reset_cb(self, **kwargs):
        self.num_ticks = 0
        keys_to_pop = []
        for cname, msg in kwargs.items():
            if msg.info.done:
                keys_to_pop.append(cname)
            else:
                kwargs[cname] = msg.msgs[0]
        [kwargs.pop(key) for key in keys_to_pop]
        self.num_resets += 1
        return self.reset(**kwargs)

    def callback_cb(self, node_tick: int, t_n: float, **kwargs: Optional[Msg]):
        self.iter_ticks += 1
        if self.log_memory and self.iter_ticks % self.print_iter == 0:
            if self.iter_start:
                # Time steps
                iter_stop = time.time()
                if self.iter_ticks > 0:
                    iter_time = float((iter_stop - self.iter_start) / self.iter_ticks)
                else:
                    iter_time = float(iter_stop - self.iter_start)

                # Memory usage request
                mem_use = (np.array(self.py.memory_info()[0:2]) / 2.0**30) * 1000  # memory use in MB...I think

                # Print info
                self.total_ticks += self.iter_ticks
                self.iter_ticks = 0

                # Log history
                if len(self.history) > 0:
                    delta_mem_rss = round(mem_use[0] - self.history[-1][3], 2)
                    delta_mem_vms = round(mem_use[1] - self.history[-1][6], 2)
                    delta_iter_time = round(iter_time - self.history[-1][9], 5)
                    cum_mem_rss = round(mem_use[0] - self.history[0][3], 2)
                    cum_mem_vms = round(mem_use[1] - self.history[0][6], 2)
                    cum_iter_time = round(iter_time - self.history[0][9], 5)
                    self.history.append(
                        [
                            self.pid,
                            self.name,
                            self.total_ticks,
                            round(mem_use[0], 1),
                            delta_mem_rss,
                            cum_mem_rss,
                            round(mem_use[1], 1),
                            delta_mem_vms,
                            cum_mem_vms,
                            iter_time,
                            delta_iter_time,
                            cum_iter_time,
                        ]
                    )
                else:
                    self.history.append(
                        [
                            self.pid,
                            self.name,
                            self.total_ticks,
                            round(mem_use[0], 1),
                            0,
                            0,
                            round(mem_use[1], 1),
                            0,
                            0,
                            iter_time,
                            0,
                            0,
                        ]
                    )
                self.backend.get_log_fn(self.log_level)("\n" + tabulate(self.history, headers=self.headers))
                # bnd.loginfo("\n" + tabulate(self.history, headers=self.headers))
            self.iter_start = time.time()
        # Only apply the callback after all pipelines have been initialized
        # Only then, the initial state has been set.
        if self.num_resets >= 1:
            self.callback(t_n)
        # Fill output msg with number of node ticks
        self.num_ticks += 1
        return dict(tick=np.array(node_tick + 1, dtype=self.dtype_tick))

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        # Set default engine params
        with spec.config as d:
            d.name = "engine"
            d.sync = True
            d.real_time_factor = 0
            d.simulate_delays = True
            d.executable = "python:=eagerx.core.executable_engine"

        # Add tick as output
        space = eagerx.Space(shape=(), dtype="int64")
        spec.add_output("tick", space=space)
        spec.config.outputs.append("tick")
        from eagerx.core.specs import EngineSpec  # noqa: F811

        return EngineSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)

    @abc.abstractmethod
    def add_object(
        self,
        spec: "ObjectSpec",
        *args: Union[bool, int, float, str, List, Dict],
        **kwargs: Union[bool, int, float, str, List, Dict],
    ) -> None:
        """Adds an object to the simulator that is interfaced by the engine.

        :param spec: The spec of the :class:`~eagerx.core.entities.Object` that is to be added.
        :param args: The engine-specific parameters that are required to add the :class:`~eagerx.core.entities.Object`.
        :param kwargs: The engine-specific parameters that are optional to add the :class:`~eagerx.core.entities.Object`.
        """
        pass

    @abc.abstractmethod
    def pre_reset(self, **states: Any) -> None:
        """
        An abstract method that resets the engine to its initial state before the start of an episode.

        .. note:: This method is called **before** every :class:`~eagerx.core.entities.EngineNode` and
                  :class:`~eagerx.core.entities.EngineState` has performed its reset,
                  but after all reset routines, implemented with :class:`~eagerx.core.entities.ResetNode`,
                  have reached their target.

                  - Can be useful for performing some preliminary actions on the simulator such as pausing before resetting
                    every :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState`.

                  - Reset the simulator state so that this state can be used in the reset of every
                    :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState`.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are sent by the environment and can be used to reset the engine at the start of an episode.
                       This can be anything, such as the dynamical properties of the simulator (e.g. friction coefficients).
        """
        pass

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the engine to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        .. note:: This method is called **after** every :class:`~eagerx.core.entities.EngineNode` and
                  :class:`~eagerx.core.entities.EngineState` has finished its reset.

                  - Can be useful for performing some final actions on the simulator such as unpausing after every
                    :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState` have reset.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are sent by the environment and can be used to reset the engine at the start of an episode.
                       This can be anything, such as the dynamical properties of the simulator (e.g. friction coefficients).
        """
        pass

    @abc.abstractmethod
    def callback(self, t_n: float) -> None:
        """
        The engine callback that is performed at the specified rate.

        This callback is steps the simulator by 1/:attr:`~eagerx.core.entities.Engine.rate`.

        .. note:: The engine does not have any outputs.
                  If you wish to broadcast other output messages based on properties of the simulator,
                  a separate :class:`~eagerx.core.entities.EngineNode` should be created.

        :param t_n: Time passed (seconds) since last reset. Increments with 1/:attr:`~eagerx.core.entities.Engine.rate`.
        """
        pass


class Object(Entity):
    """Baseclass for objects.

    Use this baseclass to implement objets that consist of sensors, actuators, and/or engine states.

    Users must call :func:`~eagerx.core.entities.Object.make` to make the object subclass' specification.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Object.make`

    For every supported :class:`~eagerx.core.entities.Engine`, an implementation method must be added.
    This method must have the same signature as :func:`~eagerx.core.entities.Object.example_engine`:

    - :func:`~eagerx.core.entities.Object.pybullet` (example)

    - :func:`~eagerx.core.entities.Object.ode_engine` (example)

    - ...
    """

    INFO = {
        "make": ["sensors", "actuators", "engine_states"],
    }

    @classmethod
    @abc.abstractmethod
    def make(cls, *args: Any, **kwargs: Any):
        """An abstract method that makes the specification (also referred to as `spec`) of this object.

        See :class:`~eagerx.core.specs.ObjectSpec` how sensor/actuator/engine state parameters can be set.

        This method should be decorated with:

        - :func:`eagerx.core.register.sensors` to register sensors.

        - :func:`eagerx.core.register.actuators` to register actuators.

        - :func:`eagerx.core.register.engine_states` to register engine states.

        :param args: Arguments to the subclass' make function.
        :param kwargs: Optional Arguments to the subclass' make function.
        :return: A (mutable) spec that can be used to build and subsequently initialize the entity (e.g. node, object, ...).
        """
        pass

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        params["config"] = dict(
            name=None,
            sensors=[],
            actuators=[],
            states=[],
            entity_id=params.pop("entity_id"),
        )
        params["sensors"] = dict()
        params["actuators"] = dict()
        params["states"] = dict()
        from eagerx.core.specs import ObjectSpec  # noqa: F811

        return ObjectSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        entity_id = spec.config.entity_id
        name = spec.config.name

        # Check that all components have a space
        for component in ["states"]:
            for cname, params in getattr(spec, component).items():
                msg = (
                    f'"{cname}" was defined as {component[:-1]} in "{name}", but its space ({type(params.space)}) '
                    f'is not a valid space. Check the spec of "{entity_id}".'
                )
                assert params.space is not None, msg

    def example_engine(self, spec: "ObjectSpec", graph: "EngineGraph") -> None:
        """An example of an engine-specific implementation of an object's registered sensors, actuators, and/or states.

        See :attr:`~eagerx.core.specs.ObjectSpec.engine` how engine specific parameters can be set/get.

        This method must be decorated with :func:`eagerx.core.register.engine` to register
        the engine implementation of the object.

        .. note:: This is an example method for documentation purposes only.

        :param spec: A (mutable) specification.
        :param graph: A graph containing the object's registered (disconnected) sensors & actuators.
                      Users should add nodes that inherit from :class:`~eagerx.core.entities.EngineNode`, and connect
                      them to the sensors & actuators. As such, the engine nodes become the *engine-specific implementation*
                      of the agnostic sensors & actuator definition.
        """
        raise NotImplementedError("This is a mock engine implementation for documentation purposes.")


class Processor(Entity):
    """Baseclass for processors.

    Use this baseclass to implement processor that preprocess an input/output message.

    This baseclass only supports one-way processing.

    Users must call :func:`~eagerx.core.entities.Processor.make` to make the subclass' specification.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Processor.make`

    - :func:`~eagerx.core.entities.Processor.initialize`

    - :func:`~eagerx.core.entities.Processor.convert`

    """

    INFO = {
        "make": [],
        "initialize": [],
        "convert": [],
    }

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def initialize(self, spec: "ProcessorSpec") -> None:
        """An abstract method to initialize the processor.

        :param spec: Specification of the processor.
        """
        pass

    @abc.abstractmethod
    def convert(self, msg: Any) -> Any:
        """An abstract method to preprocess messages.

        :param msg: Raw message.
        :return: Preprocessed message.
        """
        pass

    @classmethod
    def pre_make(cls, entity_id: str, entity_type: "Entity"):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        params["processor_type"] = params.pop("entity_type")
        params.pop("entity_id")
        from eagerx.core.specs import ProcessorSpec  # noqa: F811

        return ProcessorSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class EngineState(Entity):
    """Baseclass for engine states.

    Use this baseclass to implement engine states for an :class:`~eagerx.core.entities.Object`.

    Users must call :func:`~eagerx.core.entities.EngineState.make` to make the subclass' specification.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.EngineState.make`

    - :func:`~eagerx.core.entities.EngineState.initialize`

    - :func:`~eagerx.core.entities.EngineState.reset`

    """

    INFO = {
        "make": [],
        "initialize": [],
        "reset": [],
    }

    def __init__(
        self,
        ns,
        name,
        simulator,
        backend,
        params,
        object_params,
    ):
        #: Namespace of the environment. Can be set with the `name` argument to :class:`~eagerx.core.env.BaseEnv`.
        self.ns = ns
        #: Name of the state.
        self.name = name
        self.color = "grey"
        #: Responsible for all I/O communication within this process.
        self.backend = backend
        from eagerx.core.specs import EngineStateSpec, ObjectSpec

        self.initialize(EngineStateSpec(params), ObjectSpec(object_params), simulator)

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        params["state_type"] = params.pop("entity_type")
        params.pop("entity_id")
        from eagerx.core.specs import EngineStateSpec  # noqa: F811

        return EngineStateSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)
        return

    @abc.abstractmethod
    def initialize(self, spec: "EngineStateSpec", object_spec: "ObjectSpec", simulator: Any) -> None:
        """An abstract method to initialize the engine state.

        :param spec: The engine state specification.
        :param object_spec: The object specification to which the engine state belongs.
        :param simulator: A reference to the engine's simulator.
        """
        pass

    @abc.abstractmethod
    def reset(self, state: Any) -> None:
        """An abstract method to reset the engine state of an :class:`~eagerx.core.entities.Object`.

        :param state: The desired state that the user can specify before calling :func:`~eagerx.core.env.BaseEnv.reset`.
        """
        pass

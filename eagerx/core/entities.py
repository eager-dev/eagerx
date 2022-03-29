from typing import List, Dict, Optional, Union, Any
import gym
import abc
import inspect
import logging
import os
import time
from copy import deepcopy


import numpy as np
import psutil
import rospy
from genpy import Message
from std_msgs.msg import Bool, UInt64
from tabulate import tabulate

from eagerx.core.constants import TERMCOLOR, WARN, SILENT, process
from eagerx.core.rx_message_broker import RxMessageBroker
from eagerx.utils.node_utils import initialize_nodes, wait_for_node_initialization
from eagerx.utils.utils import Msg, initialize_state, check_valid_rosparam_type, get_param_with_blocking

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from eagerx.core.graph_engine import EngineGraph  # noqa: F401
    from eagerx.core.specs import (  # noqa: F401
        EntitySpec,
        BridgeSpec,
        NodeSpec,
        ConverterSpec,
        EngineStateSpec,
        ResetNodeSpec,
        ObjectSpec,
    )


class Entity(object):
    #: The unique entity_id provided to the :func:`eagerx.core.register.spec` decorator that decorates the
    #: :func:`~eagerx.core.entities.Node.spec` method.
    entity_id: str

    @classmethod
    def make(cls, entity_id: str, *args: Any, **kwargs: Any):
        """Makes the spec for this entity.

        The subclass must have registered its spec with the :func:`eagerx.core.register.spec` decorator.
        Then, you can make the specification of a subclass with the :func:`~eagerx.core.entities.Entity.make` method.
        Call :func:`~eagerx.core.entities.Entity.make` with :attr:`~eagerx.core.entities.Entity.entity_id`
        (used to register the spec) as the first argument,
        followed by the arguments of the subclass' :func:`~eagerx.core.entities.Node.spec` method.

        .. note:: In order for this method to find the registered :attr:`~eagerx.core.entities.Entity.entity_id`,
                  the subclass must have previously been imported somewhere at least once.

        :param entity_id: Name used to register the spec with the :func:`eagerx.core.register.spec` decorator by the subclass.
        :param args: Additional arguments to the subclass' spec function.
        :param kwargs: Additional optional arguments to the subclass' spec function.
        :return: A spec that can be used to build and subsequently initialize the entity (e.g. node, object, converter, ...).
        """
        from eagerx.core import register

        spec = register.make(cls, entity_id, *args, **kwargs)
        try:
            cls.check_spec(spec)
        except AssertionError as e:
            print(e)
            raise
        return spec

    @classmethod
    def get_spec(cls, entity_id: str, verbose: bool = True) -> inspect.Signature:
        """A helper method to print info on the spec signature of a subclass.

        :param entity_id: Name used to register the spec with the :func:`eagerx.core.register.spec` decorator by the subclass.
        :param verbose: Flag to print the signature output.
        :return: Signature of the subclass' spec. By passing the :attr:`~eagerx.core.entities.Entity.entity_id`
                 as the first argument to :func:`~eagerx.core.entities.Entity.make`,
                 together with the arguments specified by this signature, the entity can be build.
        """

        # """A helper method to get info on the signature of the subclass' spec method."""
        from eagerx.core import register

        return register.get_spec(cls, entity_id, verbose=verbose)

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        from eagerx.core.specs import EntitySpec

        return EntitySpec(dict(entity_id=entity_id, entity_type=entity_type))

    @classmethod
    def check_spec(cls, spec):
        pass

    @classmethod
    def initialize_spec(cls, spec: "EntitySpec") -> None:
        """Initializes the spec with the default config, and registered I/O (inputs, outputs, states, etc...).

        Must be called from within the subclass' spec function.

        :param spec: The spec that is to be initialized (i.e. filled) with the info registered with the various decorators from
                     :mod:`eagerx.core.register` in the subclass' class definition.
        """
        spec.initialize(cls)


class BaseNode(Entity):
    def __init__(
        self,
        ns: str,
        message_broker: RxMessageBroker,
        name: str,
        entity_id: str,
        node_type: str,
        rate: float,
        process: int,
        inputs: List[Dict],
        outputs: List[Dict],
        states: List[Dict],
        feedthroughs: List[Dict],
        targets: List[Dict],
        is_reactive: bool,
        real_time_factor: float,
        simulate_delays: bool,
        *args,
        executable=None,
        color: str = "grey",
        print_mode: int = TERMCOLOR,
        log_level: int = WARN,
        log_level_memory: int = SILENT,
        object_name: str = "",
        **kwargs,
    ):
        """
        The base class from which all (simulation) nodes and bridges inherit.

        All parameters that were uploaded to the rosparam server are stored in this object.

        Optional arguments are added, and may not necessarily be uploaded to the rosparam server.
        """
        #: Namespace of the environment. Can be set with the `name` argument to :class:`~eagerx.core.env.EagerxEnv`.
        self.ns: str = ns
        #: User specified node name. Can be set in :func:`~eagerx.core.entities.Node.spec`.
        self.name: str = name
        #: User specified node name with the namespace prepended. Cannot be modified directly.
        self.ns_name: str = "%s/%s" % (ns, name)
        #: Responsible for all I/O communication within this process.
        #: Nodes inside the same process share the same message broker. Cannot be modified.
        self.message_broker: RxMessageBroker = message_broker
        #: A unique entity_id with which the node is registered with the :func:`eagerx.core.register.spec` decorator by the subclass.
        #: Can be set in :func:`eagerx.core.register.spec` that decorates the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.entity_id: str = entity_id
        #: The class definition of the subclass. Follows naming convention *<module>/<NodeClassName>*.
        #: Cannot be modified.
        self.node_type: str = node_type
        #: Rate (Hz) at which the callback is called.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.rate: float = rate
        #: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.process: int = process
        #: Executable script that is used to launch the node (if launched in a separate process).
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.executable: str = executable
        #: Parameters for all selected inputs.
        self.inputs: List[Dict] = inputs
        #: Parameters for all selected outputs.
        self.outputs: List[Dict] = outputs
        #: Parameters for all selected states.
        self.states: List[Dict] = states
        #: Parameters for all feedthroughs corresponding to the selected outputs.
        self.feedthroughs: List[Dict] = feedthroughs
        #: Parameters for all selected targets.
        self.targets: List[Dict] = targets
        #: Flag that specifies whether we run reactive or asynchronous.
        #: Can be set in the bridge's :func:`~eagerx.core.entities.Bridge.spec`.
        self.is_reactive: bool = is_reactive
        #: A specified upper bound on the real_time factor. Wall-clock rate=real_time_factor*rate.
        #: If real_time_factor < 1 the simulation is slower than real time.
        #: Can be set in the bridge's :func:`~eagerx.core.entities.Bridge.spec`.
        self.real_time_factor: float = real_time_factor
        #: Flag that specifies whether input delays are simulated.
        #: You probably want to set this to False when running in the real-world.
        #: Can be set in the bridge's :func:`~eagerx.core.entities.Bridge.spec`.
        self.simulate_delays: bool = simulate_delays
        #: Specifies the color of logged messages & node color in the GUI.
        #: Check-out the termcolor documentation for the supported colors.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.color: str = color
        #: Specifies the different modes for printing: {1: TERMCOLOR, 2: ROS}.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.print_mode: int = print_mode
        #: Specifies the log level for this node:
        #: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.log_level: int = log_level
        effective_log_level = logging.getLogger("rosout").getEffectiveLevel()
        #: Specifies the log level for logging memory usage over time for this node:
        #: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}.
        #: Note that `log_level` has precedent over the memory level set here.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.Node.spec`.
        self.log_memory: int = effective_log_level >= log_level and log_level_memory >= effective_log_level
        self.initialize(*args, **kwargs)

    @staticmethod
    def get_msg_type(cls, component, cname):
        return cls.msg_types[component][cname]

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
            print_mode=TERMCOLOR,
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

                assert cname in c, (
                    f'Cname "{cname}" was selected for node "{name}", '
                    f'but it has no implementation. Check the spec of "{entity_id}".'
                )

        # Check that all states have a space_converter
        for cname in spec.config.states:
            sc = getattr(spec.states, cname).space_converter
            assert sc is not None, (
                f'State "{cname}" was selected for node "{name}", '
                f'but it has no space_converter. Check the spec of "{entity_id}".'
            )

    @abc.abstractmethod
    def initialize(
        self, *args: Union[bool, int, float, str, List, Dict], **kwargs: Union[bool, int, float, str, List, Dict]
    ) -> None:
        """
        An abstract method that initializes the node at run-time.

        :param args: Arguments as specified by the subclass. Only booleans, ints, floats, strings, lists, and dicts are supported.
        :param kwargs: Optional arguments as specified by the subclass. Only booleans, ints, floats, strings, lists, and dicts are supported.
        """
        pass

    def shutdown(self) -> None:
        """A method that can be overwritten to cleanly shutdown (e.g. release resources)."""
        pass


class Node(BaseNode):
    """Baseclass for nodes.

    Use this baseclass to implement nodes that will be added to the agnostic :class:`~eagerx.core.graph.Graph`.

    Users must call :func:`~eagerx.core.entities.Node.make` to make the registered node subclass'
    :func:`~eagerx.core.entities.Node.spec`. See :func:`~eagerx.core.entities.Node.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Node.spec`

    - :func:`~eagerx.core.entities.Node.initialize`

    - :func:`~eagerx.core.entities.Node.reset`

    - :func:`~eagerx.core.entities.Node.callback`

    - :func:`~eagerx.core.entities.Node.shutdown` (optional)

    Use baseclass :class:`~eagerx.core.entities.EngineNode` instead, for nodes that will be added to
    :class:`~eagerx.core.graph_engine.EngineGraph` when specifying a bridge implementation for an :class:`~eagerx.core.entities.Object`.

    Use baseclass :class:`~eagerx.core.entities.ResetNode` instead, for reset routines.
    """

    def __init__(self, **kwargs):
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
        super().__init__(**kwargs)

        # Determine all inputs with window > 0 (and skip)
        self.skipped_cbs = 0
        self.windowed = dict()
        for i in self.inputs:
            name = i["name"]
            window = i["window"]
            skip = i["skip"]
            if window > 0:
                self.windowed[name] = skip

        # If we run async *and* no msg for input with window > 0, then send None outputs.
        self.empty_outputs = dict()
        for i in self.outputs:
            name = i["name"]
            self.empty_outputs[name] = None
        for i in self.targets:
            name = i["name"]
            self.empty_outputs[name + "/done"] = Bool(data=False)

    def reset_cb(self, **kwargs: Optional[Message]):
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
                rospy.loginfo("\n" + tabulate(self.history, headers=self.headers))
            self.iter_start = time.time()

        # Skip callback if not all inputs with window > 0 have received at least one input.
        if (
            not self.is_reactive or not self.num_ticks > 1
        ):  # Larger than 1 here, because we might have already ticked, but not yet received a skipped input.
            for cname, skip in self.windowed.items():
                if len(kwargs[cname].msgs) == 0:
                    if skip and self.num_ticks == 0:
                        continue
                    else:
                        self.skipped_cbs += 1
                        rospy.logdebug(f"[{self.name}][{cname}]: skipped_cbs={self.skipped_cbs}")
                        return self.empty_outputs
        output = self.callback(t_n, **kwargs)
        self.num_ticks += 1
        return output

    def set_delay(self, delay: float, component: str, cname: str) -> None:
        """
        A method to vary the delay of an input or feedthrough.

        :param delay: A non-negative delay that can be varied at the beginning of an episode (during the reset procedure).
        :param component: Either "inputs" or "feedthroughs".
        :param cname: name of the component.
        """
        assert delay >= 0, "Delay must be non-negative."
        for i in getattr(self, component):
            if i["name"] == cname:
                i["delay"] = delay

    @staticmethod
    @abc.abstractmethod
    def spec(spec: "NodeSpec", *args: Any, **kwargs: Any) -> None:
        """An abstract method that creates the node's specification that is used to initialize the node at run-time.

        See :class:`~eagerx.core.specs.NodeSpec` for all default config parameters and how
        input/output/state parameters can be modified.

        This method must be decorated with :func:`eagerx.core.register.spec` to register the spec.

        The subclass' implementation must call :func:`~eagerx.core.entities.Node.initialize_spec` to initialize
        the spec with the default :attr:`~eagerx.core.specs.NodeSpec.config`
        and registered I/O (inputs, outputs, states, etc...).

        .. note:: Users should not call :func:`~eagerx.core.entities.Node.spec` directly to make the node's spec.
                  Instead, users should call :func:`~eagerx.core.entities.Node.make`.

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
        """
        pass

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """
        An abstract method that resets the node to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are send by the environment and can be used to reset the node at the start of an episode.
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

        # Check that there is atleast a single input & output defined.
        assert (
            len(spec.config.inputs) > 0 or name == "environment"
        ), f'Node "{name}" does not have any inputs selected. Please select at least one input when making the spec, or check the spec defined for "{entity_id}".'


class ResetNode(Node):
    """Baseclass for nodes that perform a reset routine.

    Use this baseclass to implement reset nodes that will be added to the agnostic :class:`~eagerx.core.graph.Graph`.

    Users must call :func:`~eagerx.core.entities.ResetNode.make` to make the registered reset node subclass'
    :func:`~eagerx.core.entities.ResetNode.spec`. See :func:`~eagerx.core.entities.ResetNode.make` for more info.

    .. note:: Subclasses must always have at least one target registered with the :func:`eagerx.core.register.targets` decorator.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.ResetNode.spec`

    - :func:`~eagerx.core.entities.ResetNode.initialize`

    - :func:`~eagerx.core.entities.ResetNode.reset`

    - :func:`~eagerx.core.entities.ResetNode.callback`

    - :func:`~eagerx.core.entities.ResetNode.shutdown` (optional)

    Use baseclass :class:`~eagerx.core.entities.EngineNode` instead, for nodes that will be added to
    :class:`~eagerx.core.graph_engine.EngineGraph` when specifying a bridge implementation for an :class:`~eagerx.core.entities.Object`.

    Use baseclass :class:`~eagerx.core.entities.Node` instead, for regular nodes that will be added to the
    agnostic :class:`~eagerx.core.graph.Graph`.
    """

    @staticmethod
    @abc.abstractmethod
    def spec(spec: "ResetNodeSpec", *args: Any, **kwargs: Any) -> None:
        """An abstract method that creates the engine node's specification that is used to initialize the node at run-time.

        See :class:`~eagerx.core.specs.ResetNodeSpec` for all default config parameters and how
        input/output/target/state parameters can be modified.

        This method must be decorated with :func:`eagerx.core.register.spec` to register the spec.

        The subclass' implementation must call :func:`~eagerx.core.entities.ResetNode.initialize_spec` to initialize
        the spec with the default :attr:`~eagerx.core.specs.ResetNodeSpec.config`
        and registered I/O (inputs, outputs, states, etc...).

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
        """
        pass

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the node to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are send by the environment and can be used to reset the node at the start of an episode.
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

        .. note:: This callback is skipped until the user calls :func:`~eagerx.core.env.EagerxEnv.reset`.
                  Until then, the messages coming in via the connected *feedthroughs* are fed through as
                  the outputs instead. For every registered output that was registered (& selected) with the
                  :func:`eagerx.core.register.outputs` decorator by the subclass, there must be a connected *feedthrough*.

        :param t_n: Time passed (seconds) since last reset. Increments with 1/:attr:`~eagerx.core.entities.ResetNode.rate`.
        :param inputs_and_targets: Inputs and targets that were registered (& selected) with the :func:`eagerx.core.register.inputs`
                                   and :func:`eagerx.core.register.targets` decorators by the subclass.
        :return: Dictionary with outputs that were registered (& selected) with the :func:`eagerx.core.register.outputs`
                 decorator by the subclass. In addition, the dictionary must contain a :class:`~std_msgs.msg.Bool` message
                 that states whether the request *target* was reached.
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
        for component in ["targets"]:
            for cname in spec._params["config"][component]:
                assert (
                    cname in spec._params[component]
                ), f'Cname "{cname}" was selected for node "{name}", but it has no implementation. Check the spec of "{entity_id}".'


class EngineNode(Node):
    """Baseclass for nodes that must be synchronized with respect to the simulator.

    Use this baseclass to implement nodes that will be added to an :class:`~eagerx.core.graph_engine.EngineGraph`
    when specifying a bridge implementation for an :class:`~eagerx.core.entities.Object`.

    Users must call :func:`~eagerx.core.entities.EngineNode.make` to make the registered engine node subclass'
    :func:`~eagerx.core.entities.EngineNode.spec`. See :func:`~eagerx.core.entities.EngineNode.make` for more info.

    .. note:: Engine nodes only receive a reference to the :attr:`~eagerx.core.entities.EngineNode.simulator`
              when the engine nodes are launched within
              the same process as the bridge. See :class:`~eagerx.core.constants.process` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.EngineNode.spec`

    - :func:`~eagerx.core.entities.EngineNode.initialize`

    - :func:`~eagerx.core.entities.EngineNode.reset`

    - :func:`~eagerx.core.entities.EngineNode.callback`

    - :func:`~eagerx.core.entities.EngineNode.shutdown` (optional)

    Use baseclass :class:`~eagerx.core.entities.Node` instead, for nodes that will be added to the
    agnostic :class:`~eagerx.core.graph.Graph`.

    Use baseclass :class:`~eagerx.core.entities.ResetNode` instead, for reset routines.
    """

    def __init__(self, object_name: str = None, simulator: Any = None, **kwargs: Any):
        if object_name:
            config = get_param_with_blocking(object_name)
            if config is None:
                rospy.logwarn(
                    f"Parameters for object registry request ({object_name}) not found on parameter server. "
                    f"Timeout: object ({object_name}) not registered."
                )
            try:
                bridge_config = config.pop("bridge")
            except KeyError:
                bridge_config = {}
        else:
            config, bridge_config = None, None
        #: A reference to the :attr:`~eagerx.core.entities.Bridge.simulator`. The simulator type depends on the bridge.
        #: Oftentimes, engine nodes require this reference in :func:`~eagerx.core.entities.EngineNode.callback` and/or
        #: :func:`~eagerx.core.entities.EngineNode.reset` to simulate (e.g. apply an action, extract a sensor measurement).
        #: Only available if the node was launched inside the bridge process.
        #: See :class:`~eagerx.core.constants.process` for more info.
        #: The simulator is initialized in the bridge's :func:`~eagerx.core.entities.Bridge.initialize` method.
        self.simulator: Any = simulator
        #: Engine nodes are always part of an :class:`~eagerx.core.graph_engine.EngineGraph`
        #: that corresponds to a specific bridge
        #: implementation of an :class:`~eagerx.core.entities.Object`. This a copy of that
        #: :class:`~eagerx.core.entities.Object`'s :attr:`~eagerx.core.entities.Object.config`.
        #: The parameters in :attr:`~eagerx.core.entities.Object.config` can be modified in the
        #: :class:`~eagerx.core.entities.Object`'s :func:`~eagerx.core.entities.Object.spec` method.
        self.config: Dict = config
        #: Engine nodes are always part of an :class:`~eagerx.core.graph_engine.EngineGraph` that
        #: corresponds to a specific bridge
        #: implementation of an :class:`~eagerx.core.entities.Object`. This attribute is a copy of that
        #: :class:`~eagerx.core.entities.Object`'s
        #: :attr:`~eagerx.core.entities.Object.config`.<:class:`~eagerx.core.entities.Bridge` :attr:`~eagerx.core.entities.bridge.entity_id`>.
        #: These parameters can be modified in the bridge-specific implementation of an :class:`~eagerx.core.entities.Object`.
        self.bridge_config: Dict = bridge_config

        # Call baseclass constructor (which eventually calls .initialize())
        super().__init__(**kwargs)

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the node to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        .. warning:: Avoid defining states for engine nodes, as you risk making your :class:`~eagerx.core.entities.Object`
                     non-agnostic to the environment. Instead, try to implement object states as an :class:`~eagerx.core.entities.EngineState`
                     of an :class:`~eagerx.core.entities.Object`.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are send by the environment and can be used to reset the node at the start of an episode.
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


class Bridge(BaseNode):
    """Baseclass for bridges.

    Use this baseclass to implement a bridge that interfaces the simulator.

    Users must call :func:`~eagerx.core.entities.Bridge.make` to make the registered bridge subclass'
    :func:`~eagerx.core.entities.Bridge.spec`. See :func:`~eagerx.core.entities.Bridge.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Bridge.spec`

    - :func:`~eagerx.core.entities.Bridge.initialize`

    - :func:`~eagerx.core.entities.Bridge.add_object`

    - :func:`~eagerx.core.entities.Bridge.pre_reset`

    - :func:`~eagerx.core.entities.Bridge.reset`

    - :func:`~eagerx.core.entities.Bridge.callback`

    - :func:`~eagerx.core.entities.Bridge.shutdown` (optional)
    """

    def __init__(self, target_addresses, node_names, is_reactive, real_time_factor, **kwargs):
        """
        Base class constructor.

        Note: Be sure to create a (placeholder) simulator object in the subclass' constructor, that is passed down
        to this constructor.

        :param simulator: Simulator object
        :param target_addresses: Addresses from which the bridge should expect "done" msgs.
        :param node_names: List of node names that are registered in the graph.
        :param is_reactive: Boolean flag. Specifies whether we run reactive or asynchronous.
        :param real_time_factor: Sets an upper bound of real_time factor. Wall-clock rate=real_time_factor*rate.
         If real_time_factor < 1 the simulation is slower than real time.
        :param kwargs: Arguments that are to be passed down to the node baseclass. See NodeBase for this.
        """
        #: The simulator object. The simulator depends on the bridge and should be initialized in the
        #: :func:`~eagerx.core.entities.Bridge.initialize` method. Oftentimes, engine nodes require a reference in
        #: :func:`~eagerx.core.entities.EngineNode.callback` and/or :func:`~eagerx.core.entities.EngineNode.reset`
        #: to this simulator object to simulate (e.g. apply an action, extract a sensor measurement).
        #: Engine nodes only have this reference if the node was launched inside the bridge process.
        #: See :class:`~eagerx.core.constants.process` for more info.
        self.simulator: Any = None
        self.target_addresses = target_addresses
        self.node_names = node_names

        # Check real_time_factor & reactive args
        assert is_reactive or (
            not is_reactive and real_time_factor > 0
        ), "Cannot have a real_time_factor=0 while not reactive. Will result in synchronization issues. Set is_reactive=True or real_time_factor > 0"

        # Initialized nodes
        self.sp_nodes = dict()
        self.launch_nodes = dict()
        self.num_resets = 0
        self.is_initialized = dict()

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
        super().__init__(is_reactive=is_reactive, real_time_factor=real_time_factor, **kwargs)

    def register_node(self, node_params):
        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        initialize_nodes(
            node_params,
            process.BRIDGE,
            self.ns,
            self.message_broker,
            self.is_initialized,
            sp_nodes,
            launch_nodes,
        )
        for _name, node in sp_nodes.items():
            # Set simulator
            if hasattr(node.node, "set_simulator"):
                node.node.set_simulator(self.simulator)
            # Initialize
            node.node_initialized()
        wait_for_node_initialization(self.is_initialized)
        self.sp_nodes.update(sp_nodes)
        self.launch_nodes.update(launch_nodes)
        return node_params, sp_nodes, launch_nodes

    def register_object(self, object_params, node_params, state_params):
        # Use obj_params to initialize object in simulator --> object info parameter dict is optionally added to simulation nodes
        try:
            bridge_params = object_params.pop("bridge")
        except KeyError:
            bridge_params = {}
        self.add_object(object_params, bridge_params, node_params, state_params)

        # Initialize states
        for i in state_params:
            i["state"]["name"] = i["name"]
            i["state"]["simulator"] = self.simulator
            i["state"]["config"] = object_params
            i["state"]["bridge_config"] = bridge_params
            i["state"]["ns"] = self.ns
            i["state"] = initialize_state(i["state"])

        # Initialize nodes
        sp_nodes = dict()
        launch_nodes = dict()
        node_args = dict(
            object_name=f"{self.ns}/{object_params['name']}",
            simulator=self.simulator,
        )
        initialize_nodes(
            node_params,
            process.BRIDGE,
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
        wait_for_node_initialization(self.is_initialized)
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
                rospy.loginfo("\n" + tabulate(self.history, headers=self.headers))
            self.iter_start = time.time()
        # Only apply the callback after all pipelines have been initialized
        # Only then, the initial state has been set.
        if self.num_resets >= 1:
            self.callback(t_n, **kwargs)
        # Fill output msg with number of node ticks
        self.num_ticks += 1
        return dict(tick=UInt64(data=node_tick + 1))

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        # Set default bridge params
        with spec.config as d:
            d.name = "bridge"
            d.is_reactive = True
            d.real_time_factor = 0
            d.simulate_delays = True
            d.executable = "python:=eagerx.core.executable_bridge"
        from eagerx.core.specs import BridgeSpec  # noqa: F811

        return BridgeSpec(spec.params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)

    @staticmethod
    @abc.abstractmethod
    def spec(spec: "BridgeSpec", *args: Any, **kwargs: Any):
        """An abstract method that creates the bridge's specification that is used to initialize the bridge at run-time.

        See :class:`~eagerx.core.specs.BridgeSpec` for all default config parameters.

        This method must be decorated with :func:`eagerx.core.register.spec` to register the spec.

        The subclass' implementation must call :func:`~eagerx.core.entities.Bridge.initialize_spec` to initialize
        the spec with the default :attr:`~eagerx.core.specs.BridgeSpec.config`
        and registered I/O (inputs, outputs, states, etc...).

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
        """
        pass

    @abc.abstractmethod
    def add_object(self, config: Dict, bridge_config: Dict, node_params: List[Dict], state_params: List[Dict]) -> None:
        """
        Adds an object to the simulator that is interfaced by the bridge.

        This method must be decorated with :func:`eagerx.core.register.bridge_config` to register bridge specific
        parameters that are required to add an :class:`~eagerx.core.entities.Object` to
        the bridge's :attr:`~eagerx.core.entities.Bridge.simulator`.

        :param config: The (agnostic) config of the :class:`~eagerx.core.entities.Object` that is to be added.
        :param bridge_config: The bridge-specific config of the :class:`~eagerx.core.entities.Object` that is to be added.

        :param node_params: A list containing the config of every :class:`~eagerx.core.entities.EngineNode` that represents
                            an :class:`~eagerx.core.entities.Object`'s sensor or actuator that is to be added.
        :param state_params: A list containing the parameters of every the :class:`~eagerx.core.entities.Object`'s
                             :class:`~eagerx.core.entities.EngineState` that is to be added.
        """
        pass

    @abc.abstractmethod
    def pre_reset(self, **states: Any) -> None:
        """
        An abstract method that resets the bridge to its initial state before the start of an episode.

        .. note:: This method is called **before** every :class:`~eagerx.core.entities.EngineNode` and
                  :class:`~eagerx.core.entities.EngineState` has performed its reset,
                  but after all reset routines, implemented with :class:`~eagerx.core.entities.ResetNode`,
                  have reached their target.

                  - Can be useful for performing some preliminary actions on the simulator such as pausing before resetting
                    every :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState`.

                  - Reset the simulator state so that this state can be used in the reset of every
                    :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState`.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are send by the environment and can be used to reset the bridge at the start of an episode.
                       This can be anything, such as the dynamical properties of the simulator (e.g. friction coefficients).
        """
        pass

    @abc.abstractmethod
    def reset(self, **states: Any) -> None:
        """An abstract method that resets the bridge to its initial state before the start of an episode.

        This method should be decorated with :func:`eagerx.core.register.states` to register the states.

        .. note:: This method is called **after** every :class:`~eagerx.core.entities.EngineNode` and
                  :class:`~eagerx.core.entities.EngineState` has finished its reset.

                  - Can be useful for performing some final actions on the simulator such as unpausing after every
                    :class:`~eagerx.core.entities.EngineNode` and :class:`~eagerx.core.entities.EngineState` have reset.

        :param states: States that were registered (& selected) with the :func:`eagerx.core.register.states` decorator by the subclass.
                       The state messages are send by the environment and can be used to reset the bridge at the start of an episode.
                       This can be anything, such as the dynamical properties of the simulator (e.g. friction coefficients).
        """
        pass

    @abc.abstractmethod
    def callback(self, t_n: float, **engine_node_outputs: Msg) -> None:
        """
        The bridge callback that is performed at the specified rate.

        This method should be decorated with :func:`eagerx.core.register.outputs` to register
        `tick` = :class:`std_msgs.msg.UInt64` as the only output.

        This callback is often used to step the simulator by 1/:attr:`~eagerx.core.entities.Bridge.rate`.

        .. note:: The bridge **only** outputs the registered 'tick' message after each callback. However, the user is not responsible
                  for creating this message. This is taken care of outside this method. Subclasses cannot register any
                  other output for this callback with the :func:`eagerx.core.register.outputs` decorator.
                  If you wish to broadcast other output messages based on properties of the simulator,
                  a separate :class:`~eagerx.core.entities.EngineNode` should be created.

        .. note:: The outputs of every :class:`~eagerx.core.entities.EngineNode` for every registered
                  :class:`~eagerx.core.entities.Object` are automatically registered as input to the bridge to ensure
                  input-output synchronization.

        :param t_n: Time passed (seconds) since last reset. Increments with 1/:attr:`~eagerx.core.entities.Bridge.rate`.
        :param engine_node_outputs: The outputs of every :class:`~eagerx.core.entities.EngineNode` for every registered
                                    :class:`~eagerx.core.entities.Object`.
        """
        pass


class Object(Entity):
    """Baseclass for objects.

    Use this baseclass to implement objets that consist of sensors, actuators, and/or engine states.

    Users must call :func:`~eagerx.core.entities.Object.make` to make the registered node subclass'
    :func:`~eagerx.core.entities.Object.spec`. See :func:`~eagerx.core.entities.Object.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Object.spec`

    - :func:`~eagerx.core.entities.Object.agnostic`

    For every supported :class:`~eagerx.core.entities.Bridge`, an implementation method must be added
    that has the same signature as :func:`~eagerx.core.entities.Object.example_bridge`:

    - :func:`~eagerx.core.entities.Object.pybullet` (example)

    - :func:`~eagerx.core.entities.Object.ode_bridge` (example)

    - ...
    """

    @staticmethod
    @abc.abstractmethod
    def agnostic(spec: "ObjectSpec", *args: Any, **kwargs: Any) -> None:
        """An abstract method that defines the agnostic interface of the object's sensors, actuators and/or engine states.

        See :class:`~eagerx.core.specs.ObjectSpec` how sensor/actuator/engine state parameters can be set.

        This method should be decorated with:

        - :func:`eagerx.core.register.config` to register the object's default config.

        - :func:`eagerx.core.register.sensors` to register sensors.

        - :func:`eagerx.core.register.actuators` to register actuators.

        - :func:`eagerx.core.register.engine_states` to register engine states.

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
        """
        pass

    @staticmethod
    @abc.abstractmethod
    def spec(spec: "ObjectSpec", *args: Any, **kwargs: Any) -> None:
        """An abstract method that creates the object's specification that is used to initialize the object's
        sensors, actuators, and/or engine states at run-time.

        See :attr:`~eagerx.core.specs.ObjectSpec.config` for all default config parameters.

        This method must be decorated with :func:`eagerx.core.register.spec` to register the spec.

        The subclass' implementation must call :func:`~eagerx.core.entities.Object.initialize_spec` to initialize
        the spec with the default :attr:`~eagerx.core.specs.ObjectSpec.config`.

        The subclass' implementation must also call :func:`~eagerx.core.entities.Object.agnostic` to
        set the spec with the registered sensors, actuators and engine states.

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
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

        # Check that all selected states have a space_converter
        for cname in spec.config.states:
            sc = getattr(spec.states, cname).space_converter
            assert sc is not None, (
                f'State "{cname}" was selected for node "{name}", '
                f'but it has no space_converter. Check the spec of "{entity_id}".'
            )

    def example_bridge(self, spec: "ObjectSpec", graph: "EngineGraph") -> None:
        """An example of a bridge-specific implementation of an object's registered sensors, actuators, and/or states.

        See :attr:`~eagerx.core.specs.ObjectSpec.example_bridge` how bridge specific (i.e. bridge_config)
        parameters can be set.

        This method must be decorated with :func:`eagerx.core.register.bridge` to register
        the bridge implementation of the object.

        .. note:: This is an example method for documentation purposes only.

        :param spec: A (mutable) specification.
        :param graph: A graph containing the object's registered (disconnected) sensors & actuators.
                      Users should add nodes that inherit from :class:`~eagerx.core.entities.EngineNode`, and connect
                      them to the sensors & actuators. As such, the engine nodes become the *bridge-specific implementation*
                      of the agnostic sensors & actuator definition.
        """
        raise NotImplementedError("This is a mock bridge implementation for documentation purposes.")


class BaseConverter(Entity):
    """Baseclass for converters and processors."""

    __metaclass__ = abc.ABCMeta

    def __init__(self, *args: Union[bool, int, float, str, List, Dict], **kwargs: Union[bool, int, float, str, List, Dict]):
        self.yaml_args = kwargs
        argspec = inspect.getfullargspec(self.__init__).args
        argspec.remove("self")
        for key, value in zip(argspec, args):
            self.yaml_args[key] = value
        check_valid_rosparam_type(self.yaml_args)
        self.initialize(*args, **kwargs)

    def get_yaml_definition(self) -> Dict:
        converter_type = self.__module__ + "/" + self.__class__.__name__
        yaml_dict = dict(converter_type=converter_type)
        yaml_dict.update(deepcopy(self.yaml_args))
        return yaml_dict

    @abc.abstractmethod
    def initialize(
        self, *args: Union[bool, int, float, str, List, Dict], **kwargs: Union[bool, int, float, str, List, Dict]
    ) -> None:
        """An abstract method to initialize the converter.

        :param args: Arguments as specified by the subclass. Only booleans, ints, floats, strings, lists, and dicts are supported.
        :param kwargs: Optional arguments as specified by the subclass. Only booleans, ints, floats, strings, lists, and dicts are supported.
        """
        pass

    @abc.abstractmethod
    def convert(self, msg: Any) -> Any:
        """An abstract method to convert (or process) messages.

        :param msg: Message to convert.
        :return: Converter message.
        """
        pass

    @staticmethod
    @abc.abstractmethod
    def spec(spec: "ConverterSpec", *args: Any, **kwargs: Any) -> None:
        """An abstract method that creates the converter/processor's specification
        that is used to initialize the subclass at run-time.

        See :class:`~eagerx.core.specs.ConverterSpec` how the default config parameters can be modified.

        This method must be decorated with :func:`eagerx.core.register.spec` to register the spec.

        The subclass' implementation must call :func:`~eagerx.core.entities.BaseConverter.initialize_spec` to initialize
        the spec with the default :attr:`~eagerx.core.specs.ConverterSpec.config`.

        .. note:: Users should not call :func:`~eagerx.core.entities.BaseConverter.spec` directly to make the
                  converter/processor's spec. Instead, users should call :func:`~eagerx.core.entities.BaseConverter.make`.

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
        """
        pass

    @staticmethod
    @abc.abstractmethod
    def get_opposite_msg_type(cls, msg_type: Any) -> Any:
        pass

    @classmethod
    def pre_make(cls, entity_id: str, entity_type: "Entity"):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        params["converter_type"] = params.pop("entity_type")
        params.pop("entity_id")
        from eagerx.core.specs import ConverterSpec  # noqa: F811

        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class Processor(BaseConverter):
    """Baseclass for processors.

    Use this baseclass to implement processor that do **not** change the message type.
    This baseclass only supports one-way processing.

    Users must call :func:`~eagerx.core.entities.Processor.make` to make the registered subclass'
    :func:`~eagerx.core.entities.Processor.spec`. See :func:`~eagerx.core.entities.Processor.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Processor.spec`

    - :func:`~eagerx.core.entities.Processor.initialize`

    - :func:`~eagerx.core.entities.Processor.convert`

    Subclasses must set the following static class properties:

    - :attr:`~eagerx.core.entities.Processor.MSG_TYPE`

    Use baseclass :class:`~eagerx.core.entities.SpaceConverter` instead, for converters that are used for
    actions/observations/states and change the message type.

    Use baseclass :class:`~eagerx.core.entities.Converter` instead, for message conversion
    where the message type changes. This baseclass supports two-way conversion.
    """

    #: Supported message type
    MSG_TYPE: Any

    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        if msg_type == cls.MSG_TYPE:
            return cls.MSG_TYPE
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_type "%s" is supported.'
                % (msg_type, cls.MSG_TYPE)
            )

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        from eagerx.core.specs import ConverterSpec  # noqa: F811

        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class Converter(BaseConverter):
    """Baseclass for converters.

    Use this baseclass to implement converters that change the message type.
    This baseclass supports two-way conversion.

    Users must call :func:`~eagerx.core.entities.Converter.make` to make the registered subclass'
    :func:`~eagerx.core.entities.Converter.spec`. See :func:`~eagerx.core.entities.Converter.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.Converter.spec`

    - :func:`~eagerx.core.entities.Converter.initialize`

    - :func:`~eagerx.core.entities.Converter.A_to_B`

    - :func:`~eagerx.core.entities.Converter.B_to_A`

    Subclasses must set the following static class properties:

    - :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A`

    - :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`

    .. note:: :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A` != :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`.

    Use baseclass :class:`~eagerx.core.entities.SpaceConverter` instead, for converters that are used for
    actions/observations/states and change the message type.

    Use baseclass :class:`~eagerx.core.entities.Processor` instead, for message conversion
    where the message type remains the same. This baseclass only supports one-way conversion.
    """

    #: Supported message type
    MSG_TYPE_A: Any
    #: Supported message type
    MSG_TYPE_B: Any

    __metaclass__ = abc.ABCMeta

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @staticmethod
    def get_opposite_msg_type(cls, msg_type):
        if msg_type == cls.MSG_TYPE_A:
            return cls.MSG_TYPE_B
        elif msg_type == cls.MSG_TYPE_B:
            return cls.MSG_TYPE_A
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.'
                % (msg_type, cls.MSG_TYPE_A, cls.MSG_TYPE_B)
            )

    def convert(self, msg) -> Any:
        if isinstance(msg, self.MSG_TYPE_A):
            return self.A_to_B(msg)
        elif isinstance(msg, self.MSG_TYPE_B):
            return self.B_to_A(msg)
        else:
            raise ValueError(
                'Message type "%s" not supported by this converter. Only msg_types "%s" and "%s" are supported.'
                % (type(msg), self.MSG_TYPE_A, self.MSG_TYPE_B)
            )

    @abc.abstractmethod
    def A_to_B(self, msg: Any) -> Any:
        """An abstract method to convert
        :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A` -> :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`.

        :param msg: Message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A`.
        :return: Converter message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`.
        """
        pass

    @abc.abstractmethod
    def B_to_A(self, msg) -> Any:
        """An abstract method to convert
        :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B` -> :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A`.

        :param msg: Message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`.
        :return: Converter message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A`.
        """
        pass

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        from eagerx.core.specs import ConverterSpec  # noqa: F811

        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class SpaceConverter(Converter):
    """Baseclass for space converters.

    Use this baseclass to implement converters that are used for actions/observations/states and change the message type.

    Users must call :func:`~eagerx.core.entities.Converter.make` to make the registered subclass'
    :func:`~eagerx.core.entities.Converter.spec`. See :func:`~eagerx.core.entities.Converter.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.SpaceConverter.spec`

    - :func:`~eagerx.core.entities.SpaceConverter.initialize`

    - :func:`~eagerx.core.entities.SpaceConverter.A_to_B`

    - :func:`~eagerx.core.entities.SpaceConverter.B_to_A`

    - :func:`~eagerx.core.entities.SpaceConverter.get_space`

    Subclasses must set the following static class properties:

    - :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_A`

    - :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_B`

    .. note:: :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A` != :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`.

    Use baseclass :class:`~eagerx.core.entities.Converter` instead, for message conversion
    where the message type changes without requiring gym space support. This baseclass supports two-way conversion.

    Use baseclass :class:`~eagerx.core.entities.Processor` instead, for message conversion
    where the message type remains the same. This baseclass only supports one-way conversion.
    """

    #: Supported message type
    MSG_TYPE_A: Any
    #: Supported message type
    MSG_TYPE_B: Any

    @abc.abstractmethod
    def get_space(self) -> gym.Space:
        """An abstract method that returns the OpenAI's gym space related to converted message."""
        pass

    @classmethod
    def pre_make(cls, entity_id, entity_type):
        spec = super().pre_make(entity_id, entity_type)
        params = spec.params
        from eagerx.core.specs import ConverterSpec  # noqa: F811

        return ConverterSpec(params)

    @classmethod
    def check_spec(cls, spec):
        super().check_spec(spec)


class EngineState(Entity):
    """Baseclass for engine states.

    Use this baseclass to implement engine states for an :class:`~eagerx.core.entities.Object`.

    Users must call :func:`~eagerx.core.entities.EngineState.make` to make the registered subclass'
    :func:`~eagerx.core.entities.EngineState.spec`. See :func:`~eagerx.core.entities.EngineState.make` for more info.

    Subclasses must implement the following methods:

    - :func:`~eagerx.core.entities.EngineState.spec`

    - :func:`~eagerx.core.entities.EngineState.initialize`

    - :func:`~eagerx.core.entities.EngineState.reset`

    """

    def __init__(
        self,
        ns,
        name,
        simulator,
        config,
        bridge_config,
        *args,
        color="grey",
        print_mode="termcolor",
        **kwargs,
    ):
        #: Namespace of the environment. Can be set with the `name` argument to :class:`~eagerx.core.env.EagerxEnv`.
        self.ns = ns
        self.name = name

        #: A reference to the :attr:`~eagerx.core.entities.Bridge.simulator`. The simulator type depends on the bridge.
        #: Oftentimes, engine states require this reference in :func:`~eagerx.core.entities.EngineNode.reset`
        #: to simulate the reset of an :class:`~eagerx.core.entities.Object`'s state.
        #: The simulator is initialized in the bridge's :func:`~eagerx.core.entities.Bridge.initialize` method.
        self.simulator: Any = simulator
        #: Engine states are always part of an :class:`~eagerx.core.graph_engine.EngineGraph` that
        #: corresponds to a specific bridge
        #: implementation of an :class:`~eagerx.core.entities.Object`. This attribute is a reference to that
        #: :class:`~eagerx.core.entities.Object`'s :attr:`~eagerx.core.entities.Object.config`.
        #: The parameters in :attr:`~eagerx.core.entities.Object.config` can be modified in the
        #: :class:`~eagerx.core.entities.Object`'s :func:`~eagerx.core.entities.Object.spec` method.
        self.config: Dict = config
        #: Engine states are always part of an :class:`~eagerx.core.graph_engine.EngineGraph` that
        #: corresponds to a specific bridge
        #: implementation of an :class:`~eagerx.core.entities.Object`. This attribute is a reference to that
        #: :class:`~eagerx.core.entities.Object`'s
        #: :attr:`~eagerx.core.entities.Object.config`.<:class:`~eagerx.core.entities.Bridge` :attr:`~eagerx.core.entities.bridge.entity_id`>.
        #: These parameters can be modified in the bridge-specific implementation of an :class:`~eagerx.core.entities.Object`.
        self.bridge_config: Dict = bridge_config
        #: Specifies the color of logged messages & node color in the GUI.
        #: Check-out the termcolor documentation for the supported colors.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.EngineState.spec`.
        self.color = color
        #: Specifies the different modes for printing: {1: TERMCOLOR, 2: ROS}.
        #: Can be set in the subclass' :func:`~eagerx.core.entities.EngineState.spec`.
        self.print_mode = print_mode
        self.initialize(*args, **kwargs)

    @staticmethod
    @abc.abstractmethod
    def spec(spec: "EngineStateSpec", *args: Any, **kwargs: Any) -> None:
        """An abstract method that creates the engine state specification that is used to initialize it at run-time.

        See :class:`~eagerx.core.specs.EngineStateSpec` how the default config parameters can be modified.

        This method must be decorated with :func:`eagerx.core.register.spec` to register the spec.

        The subclass' implementation must call :func:`~eagerx.core.entities.EngineState.initialize_spec` to initialize
        the spec with the default :attr:`~eagerx.core.specs.EngineStateSpec.config`.

        .. note:: Users should not call :func:`~eagerx.core.entities.EngineState.spec` directly to make the
                  engine state's spec. Instead, users should call :func:`~eagerx.core.entities.EngineState.make`.

        :param spec: A (mutable) specification.
        :param args: Additional arguments as specified by the subclass.
        :param kwargs: Additional optional arguments as specified by the subclass.
        """
        pass

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
    def initialize(self, *args: Any, **kwargs: Any) -> None:
        """An abstract method to initialize the engine state.

        :param args: Arguments as specified by the subclass. Only booleans, ints, floats, strings, lists, and dicts are supported.
        :param kwargs: Optional arguments as specified by the subclass. Only booleans, ints, floats, strings, lists, and dicts are supported.
        """
        pass

    @abc.abstractmethod
    def reset(self, state: Any, done: bool) -> None:
        """An abstract method to reset the engine state of an :class:`~eagerx.core.entities.Object`.

        :param state: The desired state that the user can specify before calling :func:`~eagerx.core.env.EagerxEnv.reset`.
        :param done: A flag whether to reset the state. If True, the state might have  already been reset by a
                     :class:`~eagerx.core.entities.ResetNode`, or the user has not specified any desired state before
                     calling :func:`~eagerx.core.env.EagerxEnv.reset`.
        """
        pass

from typing import Dict, Optional, Union, Type, TYPE_CHECKING
import gym
from yaml import dump

import eagerx
from eagerx.core.space import Space
from eagerx.core.view import SpecView
from eagerx.utils.utils import (
    replace_None,
    deepcopy,
)
from eagerx.utils.utils_sub import substitute_args


if TYPE_CHECKING:
    from eagerx.core.entities import Engine


class EntitySpec(object):
    def __init__(self, params):
        super(EntitySpec, self).__setattr__("_params", params)

    def __setattr__(self, name, value):
        raise AttributeError("You cannot set the new attributes to EntitySpec.")

    def __str__(self):
        return dump(self._params)

    @property
    @deepcopy
    def params(self):
        return self._params


class BackendSpec(EntitySpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the selected backend."""

    def initialize(self, spec_cls):
        pass

    @property
    def config(self) -> SpecView:
        """Provides an API to get/set the parameters to initialize.

        :return: (mutable) API to get/set parameters.
        """
        return SpecView(self, depth=["config"], unlocked=True)


class ProcessorSpec(EntitySpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the processor."""

    def initialize(self, spec_cls):
        pass

    @property
    def config(self) -> SpecView:
        """Provides an API to get/set the parameters to initialize.

        :return: (mutable) API to get/set parameters.
        """
        return SpecView(self, depth=[], unlocked=True)


class EngineStateSpec(EntitySpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the engine state."""

    def initialize(self, spec_cls):
        pass

    @property
    def config(self) -> SpecView:
        """Provides an API to get/set the parameters to initialize.

        :return: API to get/set parameters.
        """
        return SpecView(self, depth=[], unlocked=True)


class BaseNodeSpec(EntitySpec):
    def __init__(self, params):
        super().__init__(params)

    def _lookup(self, depth, unlocked=False):
        name = self._params["config"]["name"]
        return SpecView(self, depth=[depth], name=name, unlocked=unlocked)

    @property
    def config(self) -> SpecView:
        """Provides an API to set/get the parameters to initialize.

        The default parameters are:

        - .. py:attribute:: Spec.config.name: str

            User specified unique node name.

        - .. py:attribute:: Spec.config.rate: float

            Rate (Hz) at which the :func:`~eagerx.core.entities.Node.callback` is called.

        - .. py:attribute:: Spec.config.process: int = 0

            Process in which the node is launched. See :class:`~eagerx.core.constants.process` for all options.

        - .. py:attribute:: Spec.config.color: str = grey

            Specifies the color of logged messages & node color in the GUI.
            Check-out the termcolor documentation for the supported colors.

        - .. py:attribute:: Spec.config.print_mode: int = 1

            Specifies the different modes for printing: `{1: TERMCOLOR, 2: ROS}`.

        - .. py:attribute:: Spec.config.log_level: int = 30

            Specifies the log level for the engine: `{0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}`

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("config", unlocked=True)

    @property
    def inputs(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.inputs`.

        The mutable parameters are:

        - .. py:attribute:: Spec.inputs.<name>.window: int = 1

           A non-negative number that specifies the number of messages to pass to the node's :func:`~eagerx.core.entities.Node.callback`.

           - *window* = 1: Only the last received input message.

           - *window* = *x* > 1: The trailing last *x* received input messages.

           - *window* = 0: All input messages received since the last call to the node's :func:`~eagerx.core.entities.Node.callback`.

           .. note:: With *window* = 0, the number of input messages may vary and can even be zero.

        - .. py:attribute:: Spec.inputs.<name>.processor: ProcessorSpec = None

            A processor that preprocesses the received input message before passing it
            to the node's :func:`~eagerx.core.entities.Node.callback`.

        - .. py:attribute:: Spec.inputs.<name>.space: dict = None

            This space defines the format of valid messages.

        - .. py:attribute:: Spec.inputs.<name>.delay: float = 0.0

            A non-negative simulated delay (seconds). This delay is ignored if
            :attr:`~eagerx.core.entities.Engine.simulate_delays` = True
            in the engine's :func:`~eagerx.core.entities.Engine.spec`.

        - .. py:attribute:: Spec.inputs.<name>.skip: bool = False

            Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.Node.callback`.
            May be necessary to ensure that the connected graph is directed and acyclic.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("inputs")

    @property
    def outputs(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.outputs`.

        The mutable parameters are:

        - .. py:attribute:: Spec.outputs.<name>.processor: ProcessorSpec = None

            A processor that preprocesses the output message, returned by :func:`~eagerx.core.entities.Node.callback`,
            before publishing it.

        - .. py:attribute:: Spec.outputs.<name>.space: dict = None

            This space defines the format of valid messages.

        :return: API to get/set parameters.
        """
        return self._lookup("outputs")

    @property
    def states(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.states`.

        The mutable parameters are:

        - .. py:attribute:: Spec.states.<name>.space: dict = None

            This space defines the format of valid messages.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("states")

    def initialize(self, spec_cls):
        import eagerx.core.register as register

        try:
            params = register.LOOKUP_TYPES[spec_cls.callback]
        except KeyError:
            if spec_cls.__name__ == "EnvNode":
                params = dict()
            else:
                raise

        if "targets" in params:
            from eagerx.core.entities import ResetNode

            assert issubclass(
                spec_cls, ResetNode
            ), "You can only have targets registered for nodes that inherit from the ResetNode baseclass."
            add_ft = True
        else:
            add_ft = False

        # Set default components
        for component, cnames in params.items():
            for cname, space in cnames.items():
                if component == "outputs":
                    if cname not in self.config.outputs:
                        self.config.outputs.append(cname)
                    mapping = dict(
                        rate="$(config rate)",
                        processor=None,
                        space=space,
                    )
                    # Add feedthrough entries for each output if node is a reset node (i.e. when it has a target)
                    if add_ft:
                        mapping_ft = dict(
                            delay=0.0,
                            window=1,
                            skip=False,
                            processor=None,
                            space=space,
                            address=None,
                        )
                        with self.feedthroughs as d:
                            d[cname] = mapping_ft
                elif component == "inputs":
                    if cname not in self.config.inputs:
                        self.config.inputs.append(cname)
                    address = "engine/outputs/tick" if cname == "tick" else None
                    space = eagerx.Space(shape=(), dtype="int64") if cname == "tick" else space
                    mapping = dict(
                        delay=0.0,
                        window=1,
                        skip=False,
                        processor=None,
                        space=space,
                        address=address,
                    )
                elif component == "targets":
                    if cname not in self.config.targets:
                        self.config.targets.append(cname)
                    mapping = dict(
                        processor=None,
                        space=space,
                        address=None,
                    )
                else:
                    if cname not in self.config.states:
                        self.config.states.append(cname)
                    component = "states"
                    mapping = dict(
                        processor=None,
                        space=space,
                    )
                with getattr(self, component) as d:
                    d[cname] = mapping

    def add_input(
        self,
        cname: str,
        window: int = 1,
        delay: float = 0.0,
        skip: bool = False,
        address: str = None,
        processor: Optional[ProcessorSpec] = None,
        space: Optional[gym.spaces.Space] = None,
    ):
        mapping = dict(
            window=window,
            delay=delay,
            skip=skip,
            space=space,
            address=address,
            processor=processor.params if processor else None,
        )
        with self.inputs as d:
            d[cname] = mapping

    def add_output(
        self,
        cname: str,
        processor: Optional[ProcessorSpec] = None,
        space: Optional[gym.spaces.Space] = None,
    ):
        mapping = dict(
            rate="$(config rate)",
            space=space,
            processor=processor.params if processor else None,
        )
        with self.outputs as d:
            d[cname] = mapping

    def build(self, ns):
        params = self.params  # Creates a deepcopy
        name = self.config.name
        entity_id = self.config.entity_id

        # Replace args in .yaml
        context = {
            "ns": {"env_name": ns, "node_name": name},
            "config": params["config"],
        }
        substitute_args(params, context, only=["config", "ns"])

        # Process inputs
        inputs = []
        for cname in self.config.inputs:
            assert (
                cname in params["inputs"]
            ), f'Received unknown {"input"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            assert (
                "targets" not in params or cname not in params["targets"]
            ), f'Input "{cname}" cannot have the same cname as a target. Change either the input or target cname. Check the spec of "{name}" with entity_id "{entity_id}".'
            n = RxInput(name=cname, **params["inputs"][cname])
            inputs.append(n)

        # Process outputs
        outputs = []
        for cname in self.config.outputs:
            msg = f"The rate ({params['outputs'][cname]['rate']} Hz) set for action '{cname}' does not equal the environment rate ({self.config.rate} Hz)."
            assert params["outputs"][cname]["rate"] == self.config.rate, msg
            assert (
                cname in params["outputs"]
            ), f'Received unknown {"output"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            if "address" in params["outputs"][cname]:
                address = params["outputs"][cname].pop("address")
            else:
                address = "%s/outputs/%s" % (name, cname)
            n = RxOutput(name=cname, address=address, **params["outputs"][cname])
            outputs.append(n)

        states = []
        for cname in self.config.states:
            assert (
                cname in params["states"]
            ), f'Received unknown {"state"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
            if "address" in params["states"][cname]:  # if 'env/supervisor', the state address is pre-defined (like an input)
                n = RxState(name=cname, **params["states"][cname])
            else:
                address = "%s/states/%s" % (name, cname)
                n = RxState(name=cname, address=address, **params["states"][cname])
            states.append(n)

        targets = []
        if "targets" in self.config:
            for cname in self.config.targets:
                assert (
                    cname in params["targets"]
                ), f'Received unknown {"target"} "{cname}". Check the spec of "{name}" with entity_id "{entity_id}".'
                n = RxState(name=cname, **params["targets"][cname])
                targets.append(n)

        feedthroughs = []
        if "feedthroughs" in params:
            assert "targets" in self.config, f'No targets defined for ResetNode "{name}".'
            assert len(self.config.targets) > 0, f'No targets selected for ResetNode "{name}".'
            for cname in self.config.outputs:
                # Add output details  to feedthroughs
                assert (
                    cname in params["feedthroughs"]
                ), f'Feedthrough "{cname}" must directly correspond to a selected output. Check the spec of "{name}" with entity_id "{entity_id}".'
                params["feedthroughs"][cname]["space"] = params["outputs"][cname]["space"]
                n = RxFeedthrough(feedthrough_to=cname, **params["feedthroughs"][cname])
                feedthroughs.append(n)

        params["outputs"] = [i.build(ns=ns) for i in outputs]
        params["inputs"] = [i.build(ns=ns) for i in inputs]
        params["states"] = [i.build(ns=ns) for i in states]
        params["targets"] = [i.build(ns=ns) for i in targets]
        params["feedthroughs"] = [i.build(ns=ns) for i in feedthroughs]

        # Create rate dictionary with outputs
        chars_ns = len(ns) + 1
        rate_dict = dict()
        for i in params["outputs"]:
            assert i["rate"] is not None and isinstance(i["rate"], (int, float)) and i["rate"] > 0, (
                f'The rate of node "{name}" (and output cname "{i["name"]}") is misspecified: rate="{i["rate"]}". '
                'Make sure that it is of type(rate)=("int", "float",) and rate > 0.'
            )
            address = i["address"][chars_ns:]
            rate_dict[address] = i["rate"]  # {'rate': i['rate']}

        # Put parameters in node namespace (watch out, order of dict keys probably matters...)
        node_params = {name: params, "rate": rate_dict}
        return replace_None(node_params)


class NodeSpec(BaseNodeSpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the node.

    .. note:: You may encounter (or use) the syntax "`$(config [parameter_name])`" to couple the values of several parameters
              in the spec. This creates a coupling between parameters so that modifications to the value of one parameter
               also affect the coupled parameter value.

              For example, setting `spec.inputs.in_1.space.low = "$(config low)"` will set the value of
              `spec.inputs.in_1.space.low=spec.config.low` when the node is initialized. Hence, any change to
              `low` will also be reflected in the space parameter `low`.
    """

    pass


class ResetNodeSpec(BaseNodeSpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the node.

    .. note:: You may encounter (or use) the syntax "`$(config [parameter_name])`" to couple the values of several parameters
              in the spec. This creates a coupling between parameters so that modifications to the value of one parameter
               also affect the coupled parameter value.

              For example, setting `spec.inputs.in_1.space.low = "$(config low)"` will set the value of
              `spec.inputs.in_1.space.low=spec.config.low` when the node is initialized. Hence, any change to
              `low` will also be reflected in the space parameter `low`
    """

    @property
    def targets(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.targets`.

        The mutable parameters are:

        - .. py:attribute:: Spec.targets.<name>.processor: ProcessorSpec = None

            A processor that preprocesses the received state message before passing it
            to the node's :func:`~eagerx.core.entities.ResetNode.callback`.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("targets")

    @property
    def feedthroughs(self) -> SpecView:
        """Provides an API to set/get the parameters of a feedthrough corresponding to registered :func:`eagerx.core.register.outputs`.

        The mutable parameters are:

        - .. py:attribute:: Spec.feedthroughs.<name>.processor: ProcessorSpec = None

            A processor that preprocesses the received input message before passing it
            to the node's :func:`~eagerx.core.entities.Node.callback`.

        - .. py:attribute:: Spec.feedthroughs.<name>.space: dict = None

            This space defines the format of valid messages.

        - .. py:attribute:: Spec.feedthroughs.<name>.delay: float = 0.0

            A non-negative simulated delay (seconds). This delay is ignored if
            :attr:`~eagerx.core.entities.Engine.simulate_delays` = True
            in the engine's :func:`~eagerx.core.entities.Engine.spec`.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("feedthroughs")


class EngineSpec(BaseNodeSpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the engine."""

    @property
    def config(self) -> SpecView:
        """Provides an API to set/get the parameters to initialize.

        The default parameters are:

        - .. py:attribute:: Spec.config.rate: float

            Rate (Hz) at which the :func:`~eagerx.core.entities.Engine.callback` is called.

        - .. py:attribute:: Spec.config.process: int = 0

            Process in which the engine is launched. See :class:`~eagerx.core.constants.process` for all options.

        - .. py:attribute:: Spec.config.sync: bool = True

            Flag that specifies whether we run reactive or asynchronous.

        - .. py:attribute:: Spec.config.real_time_factor: float = 0

            A specified upper bound on the real-time factor. `Wall-clock-rate`=`real_time_factor`*`rate`.
            If `real_time_factor` < 1 the simulation is slower than real time.

        - .. py:attribute:: Spec.config.simulate_delays: bool = True

            Flag that specifies whether input delays are simulated.
            You probably want to set this to `False` when running in the real-world.

        - .. py:attribute:: Spec.config.color: str = grey

            Specifies the color of logged messages. Check-out the termcolor documentation for the supported colors.

        - .. py:attribute:: Spec.config.print_mode: int = 1

            Specifies the different modes for printing: `{1: TERMCOLOR, 2: ROS}`.

        - .. py:attribute:: Spec.config.log_level: int = 30

            Specifies the log level for the engine: `{0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}`.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("config", unlocked=True)


class ObjectSpec(EntitySpec):
    """A specification that specifies how :class:`~eagerx.core.env.BaseEnv` should initialize the object.

    .. note:: You may encounter (or use) the syntax "`$(config [parameter_name])`" to couple the values of several parameters
              in the spec. This creates a coupling between parameters so that modifications to the value of one parameter
               also affect the coupled parameter value.

              For example, setting `spec.sensors.in_1.space.low = "$(config low)"` will set the value of
              `spec.sensors.in_1.space.low=spec.config.low` when the node is initialized. Hence, any change to
              `low` will also be reflected in the space parameter `low`
    """

    def __init__(self, params):
        super().__init__(params)

    def _lookup(self, depth, unlocked=False):
        name = self._params["config"]["name"]
        return SpecView(self, depth=[depth], name=name, unlocked=unlocked)

    def gui(self, engine_cls: Type["Engine"]) -> None:
        """Opens a graphical user interface of the object's engine implementation.

        .. note:: Requires `eagerx-gui`:

        .. highlight:: python
        .. code-block:: python

            pip3 install eagerx-gui

        :param engine_id: The `entity_id` with which the object's engine implementation was registered (e.g. "PybulletEngine").
        """
        import eagerx.core.register as register

        spec_copy = ObjectSpec(self.params)
        engine_id = engine_cls.__module__ + "/" + engine_cls.__qualname__
        spec_copy._params["engine"] = {}
        graph = register.add_engine(spec_copy, engine_id)
        graph.gui()

    @property
    def engine(self) -> Union[SpecView]:
        """Provides an API to set/get the parameters of an engine-specific implementation.

        The mutable parameters are:

        - Arguments (excluding spec) of the selected engine's :func:`~eagerx.core.entities.Engine.add_object` method.

        - .. py:attribute:: Spec.engine.states.<name>: EngineState

            Link an :class:`~eagerx.core.specs.EngineState` to a registered state with :func:`eagerx.core.register.states`.

        :return: API to get/set parameters.
        """
        return SpecView(self, depth=["engine"], name=self._params["config"]["name"])

    @property
    def sensors(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.sensors`.

        The mutable parameters are:

        - .. py:attribute:: Spec.sensors.<name>.rate: float = 1.0

            Rate (Hz) at which the sensor's :func:`~eagerx.core.entities.EngineNode.callback` is called.

        - .. py:attribute:: Spec.sensors.<name>.space: dict = None

            This space defines the format of valid messages.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("sensors")

    @property
    def actuators(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.actuators`.

        The mutable parameters are:

        - .. py:attribute:: Spec.actuators.<name>.rate: float = 1.0

            Rate (Hz) at which the actuator's :func:`~eagerx.core.entities.EngineNode.callback` is called.

        - .. py:attribute:: Spec.actuators.<name>.window: int = 1

           A non-negative number that specifies the number of messages to pass to the node's
           :func:`~eagerx.core.entities.EngineNode.callback`.

           - *window* = 1: Only the last received input message.

           - *window* = *x* > 1: The trailing last *x* received input messages.

           - *window* = 0: All input messages received since the last call to the node's
             :func:`~eagerx.core.entities.EngineNode.callback`.

           .. note:: With *window* = 0, the number of input messages may vary and can even be zero.


        - .. py:attribute:: Spec.actuators.<name>.space: dict = None

            This space defines the format of valid messages.

        - .. py:attribute:: Spec.actuators.<name>.delay: float = 0.0

            A non-negative simulated delay (seconds). This delay is ignored if
            :attr:`~eagerx.core.entities.Engine.simulate_delays` = True
            in the engine's :func:`~eagerx.core.entities.Engine.spec`.

        - .. py:attribute:: Spec.actuators.<name>.skip: bool = False

            Skip the dependency on this input during the first call to the node's :func:`~eagerx.core.entities.EngineNode.callback`.
            May be necessary to ensure that the connected graph is directed and acyclic.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("actuators")

    @property
    def states(self) -> SpecView:
        """Provides an API to set/get the parameters of registered :func:`eagerx.core.register.engine_states`.

        The mutable parameters are:

        - .. py:attribute:: Spec.states.<name>.space: dict = None

            This space defines the format of valid messages.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("states")

    @property
    def config(self) -> SpecView:
        """Provides an API to set/get the parameters to initialize.

        The default parameters are:

        - Additional parameters registered with the :func:`eagerx.core.register.config` decorator.

        - .. py:attribute:: Spec.config.name: str

            User specified unique object name.

        - .. py:attribute:: Spec.config.actuators: list

            List with selected actuators. Must be a subset of the registered :func:`eagerx.core.register.actuators`.

        - .. py:attribute:: Spec.config.sensors: list

            List with selected sensors. Must be a subset of the registered :func:`eagerx.core.register.sensors`.

        - .. py:attribute:: Spec.config.states: list

            List with selected engine_states. Must be a subset of the registered :func:`eagerx.core.register.engine_states`.

        The API becomes **read-only** once the entity is added to :class:`~eagerx.core.graph.Graph`.

        :return: API to get/set parameters.
        """
        return self._lookup("config", unlocked=True)

    def initialize(self, spec_cls):
        import eagerx.core.register as register

        agnostic = register.LOOKUP_TYPES[spec_cls.make]

        # Set default components
        for component, cnames in agnostic.items():
            for cname, space in cnames.items():
                if component == "sensors":
                    mapping = dict(
                        rate=1,
                        processor=None,
                        space=space,
                    )
                elif component == "actuators":
                    mapping = dict(
                        rate=1,
                        delay=0.0,
                        window=1,
                        skip=False,
                        processor=None,
                        space=space,
                    )
                else:
                    component = "states"
                    mapping = dict(
                        processor=None,
                        space=space,
                    )
                with getattr(self, component) as d:
                    d[cname] = mapping
                # Select component per default
                if cname not in getattr(self.config, component):
                    getattr(self.config, component).append(cname)

    def _initialize_engine_config(self, engine_config):
        # Add default config
        with self.engine as d:
            d.update(engine_config)
            d["states"] = {}
            # Add all states to engine-specific params
            with d.states as s:
                for cname in self.states.keys():
                    s[cname] = None

    def _add_graph(self, graph):
        # Register EngineGraph
        nodes, actuators, sensors = graph.register()

        # Pop states that were not implemented.
        with self.engine.states as d:
            for cname in list(self.engine.states.keys()):
                if d[cname] is None:
                    d.pop(cname)

        # Set engine_spec
        with self.engine as d:
            d.actuators = actuators
            d.sensors = sensors
            d.nodes = nodes

    def _initialize_object_graph(self):
        mapping = dict()
        for component in ["sensors", "actuators"]:
            try:
                mapping[component] = getattr(self, component)
            except AttributeError:
                continue

        from eagerx.core.graph_engine import EngineGraph

        graph = EngineGraph.create(**mapping)
        return graph

    def add_engine(self, engine_id):
        # Construct context & replace placeholders
        context = {"config": self.config.to_dict()}
        substitute_args(self._params["config"], context, only=["config"])  # First resolve args within the context
        substitute_args(self._params, context, only=["config"])  # Resolve rest of params

        # Add engine entry
        import eagerx.core.register as register

        self._params["engine"] = {}
        register.add_engine(self, engine_id)

    def build(self, ns, engine_id):
        params = self.params  # Creates a deepcopy
        name = self.config.name

        # Construct context
        context = {"ns": {"env_name": ns, "obj_name": name}}
        substitute_args(params["config"], context, only=["ns"])  # First resolve args within the context
        substitute_args(params, context, only=["ns"])  # Resolve rest of params

        # Get agnostic definition
        agnostic = dict()
        for key in list(params.keys()):
            if key not in ["actuators", "sensors", "states"]:
                if key not in ["config", engine_id]:
                    params.pop(key)
                continue
            agnostic[key] = params.pop(key)

        # Get engine definition
        engine = params.pop(engine_id)
        nodes = engine.pop("nodes")
        specific = dict()
        for key in list(engine.keys()):
            if key not in ["actuators", "sensors", "states"]:
                continue
            specific[key] = engine.pop(key)

        # Replace node names
        for key in list(nodes.keys()):
            key_sub = substitute_args(key, context, only=["ns"])
            nodes[key_sub] = nodes.pop(key)

        # Sensors & actuators
        sensor_addresses = dict()
        dependencies = []
        for obj_comp in ["actuators", "sensors"]:
            for obj_cname in params["config"][obj_comp]:
                try:
                    entry_lst = specific[obj_comp][obj_cname]
                except KeyError:
                    raise KeyError(
                        f'"{obj_cname}" was selected in {obj_comp} of "{name}", but there is no implementation for it in engine "{engine_id}".'
                    )

                for entry in reversed(entry_lst):
                    node_name, node_comp, node_cname = entry["name"], entry["component"], entry["cname"]
                    obj_comp_params = agnostic[obj_comp][obj_cname]
                    node_params = nodes[node_name]

                    # Determine node dependency
                    dependencies += entry["dependency"]

                    # Set rate
                    rate = obj_comp_params["rate"]
                    msg_start = f'Different rate specified for {obj_comp[:-1]} "{obj_cname}" and enginenode "{node_name}": '
                    msg_end = "If an enginenode implements a sensor/actuator, their specified rates must be equal."
                    msg_mid = f'{node_params["config"]["rate"]} vs {rate}. '
                    assert node_params["config"]["rate"] == rate, msg_start + msg_mid + msg_end
                    for o in node_params["config"]["outputs"]:
                        msg_mid = f'{node_params["outputs"][o]["rate"]} vs {rate}. '
                        assert node_params["outputs"][o]["rate"] == rate, msg_start + msg_mid + msg_end

                    # Set component params
                    node_comp_params = nodes[node_name][node_comp][node_cname]
                    if obj_comp == "sensors":
                        # Make sure that space of node is contained within agnostic space.
                        agnostic_space = Space.from_dict(obj_comp_params["space"])
                        node_space = Space.from_dict(node_comp_params["space"])
                        msg = (
                            f"The space of EngineNode `{node_name}.{node_comp}.{node_cname}` is different "
                            f"(dtype, shape, low, or high) from the space of `{name}.{obj_comp}.{obj_cname}`: \n\n"
                            f"{node_name}.{node_comp}.{node_cname}.space={node_space} \n\n"
                            f"{name}.{obj_comp}.{obj_cname}.space={agnostic_space} \n\n"
                        )
                        assert agnostic_space.contains_space(node_space), msg
                        node_comp_params.update(obj_comp_params)
                        node_comp_params["address"] = f"{name}/{obj_comp}/{obj_cname}"
                        sensor_addresses[f"{node_name}/{node_comp}/{node_cname}"] = f"{name}/{obj_comp}/{obj_cname}"
                    else:  # Actuators
                        agnostic_space = Space.from_dict(obj_comp_params["space"])
                        node_space = Space.from_dict(node_comp_params["space"])
                        msg = (
                            f"The space of EngineNode `{node_name}.{node_comp}.{node_cname}` is different "
                            f"(dtype, shape, low, or high) from the space of `{name}.{obj_comp}.{obj_cname}`: \n\n"
                            f"{name}.{node_comp}.{node_cname}.space={node_space} \n\n"
                            f"{name}.{obj_comp}.{obj_cname}.space={agnostic_space} \n\n"
                        )
                        assert agnostic_space.contains_space(node_space), msg

                        agnostic_processor = obj_comp_params.pop("processor")
                        node_comp_params.update(obj_comp_params)

                        if agnostic_processor is not None:
                            msg = (
                                f"A processor was defined for {node_name}.{node_comp}.{node_cname}, however the engine "
                                "implementation also has a processor defined. You can only have one processor."
                            )
                            assert node_comp_params["processor"] is None, msg
                            node_comp_params["processor"] = agnostic_processor

                        # Pop rate. Actuators are more-or-less inputs so have no rate?
                        node_comp_params.pop("rate")
                        # Reassign converter in case a node provides the implementation for multiple actuators
                        obj_comp_params["processor"] = agnostic_processor

        # Get set of node we are required to launch
        dependencies = list(set(dependencies))

        # Verify that no dependency is an unlisted actuator node.
        not_selected = [cname for cname in agnostic["actuators"] if cname not in params["config"]["actuators"]]
        for cname in not_selected:
            try:
                for entry in specific["actuators"][cname]:
                    node_name, node_comp, node_cname = (entry["name"], entry["component"], entry["cname"])
                    msg = (
                        f'There appears to be a dependency on enginenode "{node_name}" for the implementation of '
                        f'engine "{engine_id}" for object "{name}" to work. However, enginenode "{node_name}" is '
                        f'directly tied to an unselected actuator "{cname}". '
                        "The actuator must be selected to resolve the graph."
                    )
                    assert node_name not in dependencies, msg
            except KeyError:
                # We pass here, because if cname is not selected, but also not implemented,
                # we are sure that there is no dependency.
                pass

        # Replace enginenode outputs that have been renamed to sensor outputs
        for node_address, sensor_address in sensor_addresses.items():
            for _, node_params in nodes.items():
                for _cname, comp_params in node_params["inputs"].items():
                    if node_address == comp_params["address"]:
                        comp_params["address"] = sensor_address

        # Create states
        states = []
        state_names = []
        obj_comp = "states"
        for obj_cname in params["config"]["states"]:
            args = agnostic[obj_comp][obj_cname]
            args["name"] = f"{name}/{obj_comp}/{obj_cname}"
            args["address"] = f"{name}/{obj_comp}/{obj_cname}"
            try:
                args["state"] = specific[obj_comp][obj_cname]
            except KeyError:
                raise KeyError(
                    f'"{obj_cname}" was selected in {obj_comp} of "{name}", but there is no implementation for it in engine "{engine_id}".'
                )
            states.append(RxEngineState(**args))
            state_names.append(f'{ns}/{args["name"]}')

        # Gather node names
        params["node_names"] = [f"{ns}/{node_name}" for node_name in list(nodes.keys()) if node_name in dependencies]
        params["state_names"] = state_names

        # Add engine
        params["engine"] = engine

        # Add agnostic definition
        params.update(**agnostic)

        # Add states
        assert "states" not in params["engine"], "The keyword `states` is reserved."
        params["engine"]["states"] = [s.build(ns) for s in states]
        nodes = [NodeSpec(params) for name, params in nodes.items() if name in dependencies]
        return {name: replace_None(params)}, nodes


# REQUIRED FOR BUILDING SPECS


class Component(object):
    def __init__(self, **kwargs):
        # Iterates over provided arguments and sets the provided arguments as class properties
        for key, value in kwargs.items():
            if key == "__class__":
                continue  # Skip if __class__ type
            setattr(self, key, value)


class RxInput(Component):
    def __init__(
        self,
        name: str,
        address: str,
        window: int = 0,
        processor: Dict = None,
        space: Dict = None,
        delay: float = 0.0,
        skip: bool = False,
        dtype: str = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxInput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        # Set dtype if not already set by source
        if params["dtype"] is None:
            params["dtype"] = params["space"]["dtype"]
        return params


class RxOutput(Component):
    def __init__(
        self,
        name: str,
        address: str,
        rate: float,
        processor: Dict = None,
        space: Dict = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxOutput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        params["dtype"] = params["space"]["dtype"]
        return params


class RxFeedthrough(Component):
    def __init__(
        self,
        address: str,
        feedthrough_to: str,
        window: int = 1,
        processor: Dict = None,
        space: Dict = None,
        delay: float = 0.0,
        skip: bool = False,
        dtype: str = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxFeedthrough, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        # Set dtype if not already set by source
        if params["dtype"] is None:
            params["dtype"] = params["space"]["dtype"]
        return params


class RxState(Component):
    def __init__(
        self,
        name: str,
        address: str,
        space: Dict,
        processor: Dict = None,
        dtype: str = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        # Set dtype if not already set by source
        if params["dtype"] is None:
            params["dtype"] = params["space"]["dtype"]
        return params


class RxEngineState(Component):
    def __init__(
        self,
        name: str,
        address: str,
        state: Dict,
        processor: Dict = None,
        space: Dict = None,
    ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop("self")
        super(RxEngineState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def build(self, ns=""):
        params = self.__dict__.copy()
        params["address"] = "/".join(filter(None, [ns, params["address"]]))
        params["dtype"] = params["space"]["dtype"]
        return params

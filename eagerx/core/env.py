# EAGERX
from eagerx.core.space import Space
from eagerx.core.specs import NodeSpec, EngineSpec, BackendSpec, RxEngineState
from eagerx.core.entities import Node
from eagerx.core.graph import Graph
from eagerx.utils.node_utils import (
    initialize_nodes,
    wait_for_node_initialization,
)
from eagerx.core.executable_node import RxNode
from eagerx.core.executable_engine import RxEngine
from eagerx.core.supervisor import Supervisor, SupervisorNode
from eagerx.core.rx_message_broker import RxMessageBroker
from eagerx.core.constants import process

# OTHER IMPORTS
import atexit
import abc
import numpy as np
from typing import List, Union, Dict, Tuple, Optional
import gym


class BaseEnv(gym.Env):
    """The base class for all EAGERx environments that follows the OpenAI gym's Env API.

    - Be sure to call :func:`super().__init__` inside the subclass' constructor with the required arguments (name, graph, etc...).

    A subclass should implement the following methods:

    - :func:`~eagerx.core.env.BaseEnv.step`: Be sure to call :func:`~eagerx.core.env.BaseEnv._step` inside this method to perform the step.

    - :func:`~eagerx.core.env.BaseEnv.reset`: Be sure to call :func:`~eagerx.core.env.BaseEnv._reset` inside this method to perform the reset.

    A subclass can optionally overwrite the following properties:

    - :attr:`~eagerx.core.env.BaseEnv.observation_space`: Per default, the observations, registered in the graph, are taken.

    - :attr:`~eagerx.core.env.BaseEnv.action_space`: Per default, the actions, registered in the graph, are taken.
    """

    def __init__(
        self, name: str, rate: float, graph: Graph, engine: EngineSpec, backend: BackendSpec, force_start: bool
    ) -> None:
        """Initializes an environment with EAGERx dynamics.

        :param name: The name of the environment. Everything related to this environment
                     (parameters, topics, nodes, etc...) will be registered under namespace: "`/name`".
        :param rate: The rate (Hz) at which the environment will run.
        :param graph: The graph consisting of nodes and objects that describe the environment's dynamics.
        :param engine: The physics engine that will govern the environment's dynamics.
                       For every :class:`~eagerx.core.entities.Object` in the graph,
                       the corresponding engine implementations is chosen.
        :param backend: The backend that will govern the communication for this environment.
        :param force_start: If there already exists an environment with the same name, the existing environment is
                            first shutdown by calling the :func:`~eagerx.core.env.BaseEnv` method before initializing this
                            environment.
        """
        assert "/" not in name, 'Environment name "%s" cannot contain the reserved character "/".' % name
        self.name = name
        self.ns = "/" + name
        self.rate = rate
        self.initialized = False
        self.has_shutdown = False

        # todo: [OPTIONAL]graph._remove_engine(..) --> resubstitute all objects from backup (how to reconnect targets to engine_states?).
        # todo: [OPTIONAL] graph._agnostic_object(name="obj_name") --> keep track of nodes added
        # todo: [OPTIONAL] Add engine to EngineGraph & automatically connect tick if EngineNode is added.
        # todo: [OPTIONAL] Add legend to dag graphs.
        # todo: [OPTIONAL] Add engine: EngineSpec as argument object engine methods.
        # todo: [OPTIONAL] check arguments of engine_spec.add_object(...) are arguments of engine.add_object(...).
        # todo: [OPTIONAL] No dynamic pipeline initialization bridge
        # todo: [DONE] graph._add_engine(..) --> allow only a single engine
        # todo: [DONE] graph._substitute_all_objects()
        # todo: [DONE] graph._substitute_object(name="obj_name")
        # todo: [DONE] add engine.objects.obj_name.engine_states
        # todo: [DONE] add engine.objects.obj_name.add_object
        # todo: [DONE] add engine.objects.obj_name.nodes
        # todo: [DONE] update graph.check_valid_graph --> substitute environment for env/actions, env/observations.
        # todo: [DONE] remove object_spec from EngineNode.initialize, EngineState.initialize signatures.
        # todo: [DONE] only pass simulator[object_name] and add key simulator[object_name][name] = object_name
        # todo: [DONE] refactor all pybullet engine
        # todo: [DONE] Refactor EngineODE
        # todo: [DONE] Add to engine.build() a stage where engine_states are converted to RxEngineState (aka Object.build()).
        # todo: [DONE] Refactor RealEngine, eagerx_reality.enginenodes.py
        # todo: [DONE] add objects before launching nodes.
        # todo: [DONE] launch engine states before launching nodes.
        # todo: [DONE] rx_operators.get_object_params --> get params from engine spec instead of object spec
        # todo: [DONE] EngineNode.__init__ --> remove `object_name` argument
        # todo: [DONE] Engine nodes must be launched by Engine as an EngineNode (to get reference to simulator).
        # todo: remove dead code (ActionsNode, ObservationsNode, Graph.gui(), Graph._substitute_environment_node())
        # todo: remove dead code (EngineGraph.gui())
        # todo: update docs (object naming conventions, object_spec, add_object, engine.objects, external launching object name)
        # todo: update colab notebooks (engine states, engine nodes)
        # todo: Allow gui to visualize with environment node.

        # Register graph (unlinks the specs, reloads entities) .
        self.graph, environment, engine, nodes, self.render_node = graph.register(engine)

        # Initialize backend
        self.backend = self._init_backend(backend)

        # Check if there already exists an environment
        self._shutdown_srv = self.backend.register_environment(self.ns, force_start, self.shutdown)

        # Delete pre-existing parameters
        self.backend.delete_param(f"/{self.name}", level=2)

        # Upload log_level
        self.backend.upload_params(self.ns, {"log_level": self.backend.log_level})

        # Initialize message broker
        self.mb = RxMessageBroker(owner="%s/%s" % (self.ns, "env"), backend=self.backend)

        # Initialize supervisor node
        self.supervisor_node, self.supervisor = self._init_supervisor(engine, nodes)
        self._is_initialized = self.supervisor_node.is_initialized

        # Initialize engine
        self._init_engine(engine, nodes)

        # Create environment node
        self.env_node, self.env = self._init_environment(environment, self.supervisor_node, self.mb)

        # Register render node
        if self.render_node:
            nodes = [self.render_node] + nodes

        # Register nodes
        self._register_nodes(nodes)

        # Implement clean up
        atexit.register(self.shutdown)

    def _init_backend(self, backend: BackendSpec):
        # Initialize backend
        from eagerx.core.entities import Backend

        backend = Backend.from_cmd(self.ns, backend.config.entity_id, backend.config.log_level)

        return backend

    def _init_supervisor(self, engine: EngineSpec, nodes: List[NodeSpec]):
        # Initialize supervisor
        supervisor = self._create_supervisor()

        # Get all states from objects & nodes
        for i in [engine] + nodes:
            for cname in i.params["config"]["states"]:
                entity_name = i.config.name
                name = f"{entity_name}/{cname}"
                address = f"{entity_name}/states/{cname}"
                processor = None  # Only add processor (i.params["states"][cname]["processor"]) once at input side.
                space = i.params["states"][cname]["space"]

                assert (
                    name not in supervisor.params["states"]
                ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                mapping = dict(address=address, processor=processor, space=space)
                with supervisor.states as d:
                    d[name] = mapping
                supervisor.config.states.append(name)

        for entity_name, obj in engine.objects.items():
            for cname, state in obj.engine_states.items():
                name = f"{entity_name}/{cname}"
                address = f"{entity_name}/states/{cname}"
                processor = None  # Only add processor (i.params["states"][cname]["processor"]) once at input side.
                space = state["space"].to_dict()

                assert (
                    name not in supervisor.params["states"]
                ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                mapping = dict(address=address, processor=processor, space=space)
                with supervisor.states as d:
                    d[name] = mapping
                supervisor.config.states.append(name)

        # Get info from engine on reactive properties
        sync = engine.config.sync
        real_time_factor = engine.config.real_time_factor
        simulate_delays = engine.config.simulate_delays

        # Create supervisor node
        name = supervisor.config.name
        supervisor.config.rate = self.rate
        supervisor_params = supervisor.build(ns=self.ns)
        self.backend.upload_params(self.ns, supervisor_params)
        rx_supervisor = Supervisor(
            "%s/%s" % (self.ns, name),
            self.mb,
            sync,
            real_time_factor,
            simulate_delays,
        )
        rx_supervisor.node_initialized()

        # Connect io
        self.mb.connect_io()
        return rx_supervisor.node, rx_supervisor

    def _init_engine(self, engine: EngineSpec, nodes: List[NodeSpec]) -> None:
        # Check that reserved keywords are not already defined.
        assert (
            "node_names" not in engine.params["config"]
        ), f'Keyword "{"node_names"}" is a reserved keyword within the engine params and cannot be used twice.'
        assert (
            "target_addresses" not in engine.params["config"]
        ), f'Keyword "{"target_addresses"}" is a reserved keyword within the engine params and cannot be used twice.'
        assert (
            not engine.params["config"]["process"] == process.ENGINE
        ), "Cannot initialize the engine inside the engine process, because it has not been launched yet. You can choose process.{ENVIRONMENT, EXTERNAL, NEW_PROCESS}."

        # Extract node_names
        node_names = ["environment", "env/supervisor"]
        target_addresses = []
        for i in nodes:
            node_names.append(i.params["config"]["name"])
            if "targets" in i.params["config"]:
                for cname in i.params["config"]["targets"]:
                    address = i.params["targets"][cname]["address"]
                    target_addresses.append(address)
        with engine.config as d:
            d.node_names = node_names
            d.target_addresses = target_addresses

        # Convert engine states
        for name, obj in engine.objects.items():
            for cname, view in obj.engine_states.items():
                address = f"{name}/states/{cname}"
                processor = view.processor.to_dict() if view.processor else None
                spec = RxEngineState(cname, address, view.state.to_dict(), processor, view.space.to_dict())
                obj.engine_states[cname] = spec.build(ns=self.ns)

        initialize_nodes(
            engine,
            process.ENVIRONMENT,
            self.ns,
            self.mb,
            self.supervisor_node.is_initialized,
            self.supervisor_node.sp_nodes,
            self.supervisor_node.launch_nodes,
            rxnode_cls=RxEngine,
        )
        wait_for_node_initialization(self._is_initialized, self.backend)  # Proceed after engine is initialized

    def _init_environment(self, environment: NodeSpec, supervisor_node, message_broker):
        # Check that env has at least one input & output.
        assert (
            len(environment.params["config"]["inputs"]) > 0
        ), f'Environment "{self.name}" must have at least one input (i.e. input).'
        assert (
            len(environment.params["config"]["outputs"]) > 0
        ), f'Environment "{self.name}" must have at least one action (i.e. output).'

        # Check that all observation addresses are unique
        addresses_obs = [environment.params["inputs"][cname]["address"] for cname in environment.params["config"]["inputs"]]
        len(set(addresses_obs)) == len(
            addresses_obs
        ), "Duplicate observations found: %s. Make sure to only have unique observations" % (
            set([x for x in addresses_obs if addresses_obs.count(x) > 1])
        )

        # Create env node
        environment.config.rate = self.rate
        name = environment.config.name
        env_params = environment.build(ns=self.ns)
        self.backend.upload_params(self.ns, env_params)
        rx_env = RxNode(name="%s/%s" % (self.ns, name), message_broker=message_broker)
        rx_env.node_initialized()

        # Set env_node in supervisor
        supervisor_node.set_environment(rx_env.node)

        return rx_env.node, rx_env

    @property
    def observation_space(self) -> gym.spaces.Space:
        """The Space object corresponding to valid observations.

        Per default, the observation space of all registered observations in the graph is used.
        """
        return self._observation_space

    @property
    def action_space(self) -> gym.spaces.Space:
        """The Space object corresponding to valid actions

        Per default, the action space of all registered actions in the graph is used.
        """
        return self._action_space

    @property
    def state_space(self) -> gym.spaces.Dict:
        """Infers the state space from the space of every state.

        This space defines the format of valid states that can be set before the start of an episode.

        :returns: A dictionary with *key* = *state* and *value* = :class:`Space`.
        """
        state_space = dict()
        for name, buffer in self.supervisor_node.state_buffer.items():
            state_space[name] = buffer["space"]
        return gym.spaces.Dict(spaces=state_space)

    @property
    def _observation_space(self) -> gym.spaces.Dict:
        """Infers the observation space from the space of every observation.

        This space defines the format of valid observations.

        .. note:: Observations with :attr:`~eagerx.core.specs.RxInput.window` = 0 are excluded from the observation space.
                  For observations with :attr:`~eagerx.core.specs.RxInput.window` > 1,
                  the observation space is duplicated :attr:`~window` times.

        :returns: A dictionary with *key* = *observation* and *value* = :class:`Space`.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        observation_space = dict()
        for name, buffer in self.env_node.observation_buffer.items():
            space = buffer["space"]
            if not buffer["window"] > 0:
                continue
            if isinstance(space, gym.spaces.Discrete):
                stacked_space = gym.spaces.MultiDiscrete([space.n] * buffer["window"])
            else:
                low = np.repeat(space.low[np.newaxis, ...], buffer["window"], axis=0)
                high = np.repeat(space.high[np.newaxis, ...], buffer["window"], axis=0)
                stacked_space = gym.spaces.Box(low=low, high=high, dtype=space.dtype)
            observation_space[name] = stacked_space
        return gym.spaces.Dict(spaces=observation_space)

    @property
    def _action_space(self) -> gym.spaces.Dict:
        """Infers the action space from the space of every action.

        This space defines the format of valid actions.

        :returns: A dictionary with *key* = *action* and *value* = :class:`Space`.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        action_space = dict()
        for name, buffer in self.env_node.action_buffer.items():
            action_space[name] = buffer["space"]
        return gym.spaces.Dict(spaces=action_space)

    def _set_action(self, action) -> None:
        # Set actions in buffer
        for name, buffer in self.env_node.action_buffer.items():
            assert not self.supervisor_node.sync or name in action, (
                'Action "%s" not specified. Must specify all actions in action_space if running reactive.' % name
            )
            if name in action:
                buffer["msg"] = action[name]

    def _set_state(self, state) -> None:
        # Set states in buffer
        for name, msg in state.items():
            assert name in self.supervisor_node.state_buffer, 'Cannot set unknown state "%s".' % name
            self.supervisor_node.state_buffer[name]["msg"] = msg

    def _get_observation(self) -> Dict:
        # Get observations from buffer
        observation = dict()
        for name, buffer in self.env_node.observation_buffer.items():
            observation[name] = buffer["msgs"]
        return observation

    def _initialize(self, states: Dict) -> None:
        assert not self.initialized, "Environment already initialized. Cannot re-initialize pipelines. "

        # Set desired reset states
        self._set_state(states)

        # Wait for nodes to be initialized
        [node.node_initialized() for name, node in self.supervisor_node.sp_nodes.items()]
        wait_for_node_initialization(self._is_initialized, self.backend)

        # Initialize single process communication
        self.mb.connect_io(print_status=True)

        self.backend.logdebug("Nodes initialized.")

        # Perform first reset
        self.supervisor_node.reset()

        # Nodes initialized
        self.initialized = True
        self.backend.loginfo("Communication initialized.")

    def _shutdown(self):
        if not self.has_shutdown:
            self._shutdown_srv.unregister()
            for address, node in self.supervisor_node.launch_nodes.items():
                self.backend.logdebug(f"[{self.name}] Send termination signal to '{address}'.")
                node.terminate()
            for _, rxnode in self.supervisor_node.sp_nodes.items():
                rxnode: RxNode
                if not rxnode.has_shutdown:
                    self.backend.logdebug(f"[{self.name}][{rxnode.name}] Shutting down.")
                    rxnode.node_shutdown()
            if not self.supervisor.has_shutdown:
                self.supervisor.node_shutdown()
            if not self.env.has_shutdown:
                self.env.node_shutdown()
            self.mb.shutdown()
            self.backend.delete_param(f"/{self.name}", level=1)
            self.backend.shutdown()
            self.has_shutdown = True

    def _register_nodes(self, nodes: Union[List[NodeSpec], NodeSpec]) -> None:
        assert not self.has_shutdown, "This environment has been shutdown."
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        if not isinstance(nodes, list):
            nodes = [nodes]

        # Register nodes
        [self.supervisor_node.register_node(n) for n in nodes]

    @staticmethod
    def _create_supervisor():
        entity_type = f"{SupervisorNode.__module__}/{SupervisorNode.__name__}"
        supervisor = Node.pre_make("N/a", entity_type)
        supervisor.add_output("step", space=Space(shape=(), dtype="int64"))

        supervisor.config.name = "env/supervisor"
        supervisor.config.color = "yellow"
        supervisor.config.process = process.ENVIRONMENT
        supervisor.config.outputs = ["step"]
        return supervisor

    def _reset(self, states: Dict) -> Dict:
        """A private method that should be called within :func:`~eagerx.core.env.BaseEnv.reset()`.

        :param states: The desired states to be set before the start an episode.
                       May also be an (empty) subset of registered states if not all states require a reset.
        :returns: The initial observation.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        # Initialize environment
        if not self.initialized:
            self._initialize(states)

        # Set desired reset states
        self._set_state(states)

        # Perform reset
        self.supervisor_node.reset()
        obs = self._get_observation()
        return obs

    @abc.abstractmethod
    def reset(self) -> Union[Dict, np.ndarray]:
        """An abstract method that resets the environment to an initial state and returns an initial observation.

        .. note:: To reset the graph, the private method :func:`~eagerx.core.env.BaseEnv._reset` must be called with the
                   desired initial states. The spaces of all states (of Objects and Nodes in the graph) are stored in
                  :func:`~eagerx.core.env.BaseEnv.state_space`.

        :returns: The initial observation that is complies with the :func:`~eagerx.core.env.BaseEnv.observation_space`.
        """
        pass

    def _step(self, action: Dict) -> Dict:
        """A private method that should be called within :func:`~eagerx.core.env.BaseEnv.step()`.

        :param action: The actions to be applied in the next timestep.
                       Should include all registered actions.
        :returns: The observation of the current timestep that comply with the graph's observation space.
        """
        # Check that nodes were previously initialized.
        assert self.initialized, "Not yet initialized. Call .reset() before calling .step()."
        assert not self.has_shutdown, "This environment has been shutdown."

        # Set actions in buffer
        self._set_action(action)

        # Call step
        self.supervisor_node.step()
        return self._get_observation()

    @abc.abstractmethod
    def step(self, action: Union[Dict, np.ndarray]) -> Tuple[Union[Dict, np.ndarray], float, bool, Dict]:
        """An abstract method that runs one timestep of the environment's dynamics.

        .. note:: To run one timestep of the graph dynamics (that essentially define the environment dynamics),
                  this method must call the private method :func:`~eagerx.core.BaseEnv._step` with the actions that comply
                  with :attr:`~eagerx.core.BaseEnv._action_space`.

        When the end of an episode is reached, the user is responsible for calling :func:`~eagerx.core.BaseEnv.reset`
        to reset this environment's state.

        :params action: Actions provided by the agent. Should comply with the :func:`~eagerx.core.env.BaseEnv.action_space`.

        :returns: A tuple (observation, reward, done, info).

                  - observation: Observations of the current timestep that comply with
                                 the :func:`~eagerx.core.env.BaseEnv.observation_space`.

                  - reward: amount of reward returned after previous action

                  - done: whether the episode has ended, in which case further step() calls will return undefined results

                  - info: contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        pass

    def gui(self):
        """Opens the gui of the graph that was used to initialize this environment."""
        self.graph.gui()

    def render(self, mode: str = "human") -> Optional[np.ndarray]:
        """A method to start rendering (i.e. open the render window).

        A bool message to topic address ":attr:`~eagerx.core.env.BaseEnv.name` */env/render/toggle*",
        which toggles the rendering on/off.

        :param mode: - human: render and return nothing. Usually for human consumption.
                     - rgb_array: Return a numpy.ndarray with shape (x, y, 3),
                       representing RGB values for an x-by-y pixel image, suitable
                       for turning into a video.
        :returns: Optionally, a rgb_array if mode=rgb_array.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        if self.render_node:
            if mode == "human":
                self.supervisor_node.start_render()
            elif mode == "rgb_array":
                self.supervisor_node.start_render()
                img = self.supervisor_node.get_last_image()
                return img
            else:
                raise ValueError('Render mode "%s" not recognized.' % mode)
        else:
            self.backend.logwarn_once("No render node active, so not rendering.")
            if mode == "rgb_array":
                return np.empty((0, 0, 3), dtype="uint8")
            else:
                return

    def close(self):
        """A method to stop rendering (i.e. close the render window).

        A bool message to topic address ":attr:`~eagerx.core.env.BaseEnv.name` */env/render/toggle*",
        which toggles the rendering on/off.

        .. note:: Depending on the source node that is producing the images that are rendered,
                  images may still be produced, even when the render window is not visible.
                  This may add computational overhead and influence the run speed.

                  Optionally, users may subscribe to topic address ":attr:`~eagerx.core.env.BaseEnv.name` */env/render/toggle*"
                  in the node that is producing the images to stop the production and output empty images instead.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        self.supervisor_node.stop_render()

    def shutdown(self):
        """A method to shutdown the environment.

        - Clear the parameters on the ROS parameter under the namespace /:attr:`~eagerx.core.env.BaseEnv.name`.

        - Close nodes (i.e. release resources and perform :class:`~eagerx.core.entities.Node.close` procedure).

        - Unregister topics that supplied the I/O communication between nodes.
        """
        self._shutdown()

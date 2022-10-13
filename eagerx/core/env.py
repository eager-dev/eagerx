from eagerx.core.space import Space
from eagerx.core.entities import Backend
from eagerx.core.specs import NodeSpec, EngineSpec, BackendSpec, BaseNodeSpec
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
from eagerx.core.constants import ENVIRONMENT, NEW_PROCESS, EXTERNAL, BackendException

# OTHER IMPORTS
import copy
import atexit
import abc
import numpy as np
import functools
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
        self, name: str, rate: float, graph: Graph, engine: EngineSpec, backend: BackendSpec = None, force_start: bool = True
    ) -> None:
        """Initializes  an environment with EAGERx dynamics.

        :param name: The name of the environment. Everything related to this environment
                     (parameters, topics, nodes, etc...) will be registered under namespace: "`/name`".
        :param rate: The rate (Hz) at which the environment will run.
        :param graph: The graph consisting of nodes and objects that describe the environment's dynamics.
        :param engine: The physics engine that will govern the environment's dynamics.
                       For every :class:`~eagerx.core.entities.Object` in the graph,
                       the corresponding engine implementations is chosen.
        :param backend: The backend that will govern the communication for this environment.
                        Per default, the :class:`~eagerx.backends.single_process.SingleProcess` backend is used.
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
        self._is_initialized = dict()
        self._launch_nodes = dict()
        self._sp_nodes = dict()

        # Initialize backend
        if backend is None:
            from eagerx.backends.single_process import SingleProcess

            backend = SingleProcess.make()
        self.backend = Backend.from_cmd(self.ns, backend.config.entity_id, backend.config.log_level,
                                        main=True,
                                        real_time_factor=engine.config.real_time_factor,
                                        sync=engine.config.sync,
                                        simulate_delays=engine.config.sync)

        # Check if there already exists an environment
        self._shutdown_srv = self.backend.register_environment(self.ns, force_start, self.shutdown)

        # Delete pre-existing parameters
        self.backend.delete_param(f"/{self.name}", level=2)

        # Upload relevant run-time settings
        self.backend.upload_params(self.ns, {"log_level": self.backend.log_level,
                                             "ts_init": self.backend.ts_init,
                                             "real_time_factor": self.backend.real_time_factor,
                                             "sync": self.backend.sync,
                                             "simulate_delays": self.backend.simulate_delays})

        # Initialize message broker
        self.mb = RxMessageBroker(owner="%s/%s" % (self.ns, "env"), backend=self.backend)

        # Register graph (returns unlinked specs with original graph & reloads entities).
        if engine is None:
            environment, engine, nodes, render = Graph._get_all_node_specs(graph._state)
            self.graph = graph
        else:
            self.graph, environment, engine, nodes, self.render_node = graph.register(rate, engine)
        self.rate = rate if rate else environment.config.rate

        # Create supervisor spec (adds addresses of (engine)states to supervisor)
        supervisor = self._create_supervisor(environment, engine, nodes)

        # Upload parameters
        self._upload_params(self.ns, self.backend, [supervisor, environment, engine] + nodes)

        # Initialize environment
        self.environment_node = RxNode(name="%s/%s" % (self.ns, environment.config.name), message_broker=self.mb)
        self.environment_node.node_initialized()
        self.environment = self.environment_node.node

        # Initialize nodes
        initialize_nodes(nodes, ENVIRONMENT, self.ns, self.mb, self._is_initialized, self._sp_nodes, self._launch_nodes)

        # Initialize engine
        initialize_nodes(
            engine,
            ENVIRONMENT,
            self.ns,
            self.mb,
            self._is_initialized,
            self._sp_nodes,
            self._launch_nodes,
            rxnode_cls=RxEngine,
        )

        # Initialize supervisor node
        self.supervisor_node = Supervisor("%s/%s" % (self.ns, supervisor.config.name), self.mb, self.environment)
        self.supervisor_node.node_initialized()
        self.supervisor = self.supervisor_node.node
        self.mb.connect_io()

        # Implement clean up
        atexit.register(self.shutdown)

    @staticmethod
    def _create_supervisor(environment: NodeSpec, engine: EngineSpec, nodes: List[NodeSpec]) -> BaseNodeSpec:
        entity_type = f"{SupervisorNode.__module__}/{SupervisorNode.__name__}"
        spec = Node.pre_make("N/a", entity_type)
        spec.add_output("step", space=Space(shape=(), dtype="int64"))

        spec.config.rate = environment.config.rate
        spec.config.name = "env/supervisor_node"
        spec.config.color = "yellow"
        spec.config.process = process.ENVIRONMENT
        spec.config.outputs = ["step"]

        # Get all states from all nodes
        for i in [environment, engine] + nodes:
            for cname in i.params["config"]["states"]:
                entity_name = i.config.name
                name = f"{entity_name}/{cname}"
                address = f"{entity_name}/states/{cname}"
                processor = None  # Only add processor (i.params["states"][cname]["processor"]) once at input side.
                space = i.params["states"][cname]["space"]

                assert (
                    name not in spec.params["states"]
                ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                mapping = dict(address=address, processor=processor, space=space)
                with spec.states as d:
                    d[name] = mapping
                spec.config.states.append(name)

        # Get all engine states
        for entity_name, obj in engine.objects.items():
            for cname, state in obj.engine_states.items():
                name = f"{entity_name}/{cname}"
                address = f"{entity_name}/states/{cname}"
                processor = None  # Only add processor (i.params["states"][cname]["processor"]) once at input side.
                space = state["space"].to_dict()

                assert (
                    name not in spec.params["states"]
                ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                mapping = dict(address=address, processor=processor, space=space)
                with spec.states as d:
                    d[name] = mapping
                spec.config.states.append(name)

        return spec

    @staticmethod
    def _upload_params(ns: str, backend: Backend, nodes: List[BaseNodeSpec]) -> None:
        for node in nodes:
            name = node.config.name
            msg = f"Node name '{ns}/{node.config.name}' already exists on the parameter server. Node names must be unique."
            assert backend.get_param(f"{ns}/rate/{name}", None) is None, msg
            # If no backend multiprocessing support, overwrite NEW_PROCESS to ENVIRONMENT
            if node.config.process == NEW_PROCESS and not backend.MULTIPROCESSING_SUPPORT:
                backend.logwarn_once(
                    f"Backend '{backend.BACKEND}' does not support multiprocessing, "
                    "so all nodes are launched in the ENVIRONMENT process."
                )
                node.config.process = ENVIRONMENT
            # Raise error if external process is not supported
            elif node.config.process == EXTERNAL and not backend.DISTRIBUTED_SUPPORT:
                raise BackendException(
                    f"Backend '{backend.BACKEND}' does not support distributed computation. "
                    f"Therefore, this backend is incompatible with node '{name}', "
                    f"because {name}.config.process=EXTERNAL."
                )
            params = node.build(ns=ns)
            backend.upload_params(ns, params)

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
        for name, buffer in self.supervisor.state_buffer.items():
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
        for name, buffer in self.environment.observation_buffer.items():
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
        for name, buffer in self.environment.action_buffer.items():
            action_space[name] = buffer["space"]
        return gym.spaces.Dict(spaces=action_space)

    def _set_action(self, action) -> None:
        # Set actions in buffer
        for name, buffer in self.environment.action_buffer.items():
            assert not self.supervisor.sync or name in action, (
                'Action "%s" not specified. Must specify all actions in action_space if running reactive.' % name
            )
            if name in action:
                buffer["msg"] = action[name]

    def _set_state(self, state) -> None:
        # Set states in buffer
        for name, msg in state.items():
            assert name in self.supervisor.state_buffer, 'Cannot set unknown state "%s".' % name
            self.supervisor.state_buffer[name]["msg"] = msg

    def _get_observation(self) -> Dict:
        # Get observations from buffer
        observation = dict()
        for name, buffer in self.environment.observation_buffer.items():
            observation[name] = buffer["msgs"]
        return observation

    def _initialize(self, states: Dict) -> None:
        assert not self.initialized, "Environment already initialized. Cannot re-initialize pipelines. "

        # Set desired reset states
        self._set_state(states)

        # Wait for nodes to be initialized
        [node.node_initialized() for name, node in self._sp_nodes.items()]
        wait_for_node_initialization(self._is_initialized, self.backend)

        # Initialize communication within this process
        self.mb.connect_io(print_status=True)

        self.backend.logdebug("Nodes initialized.")

        # Perform first reset
        self.supervisor.reset()

        # Nodes initialized
        self.initialized = True
        self.backend.loginfo("Communication initialized.")

    def _shutdown(self):
        if not self.has_shutdown:
            self._shutdown_srv.unregister()
            for address, node in self._launch_nodes.items():
                self.backend.logdebug(f"[{self.name}] Send termination signal to '{address}'.")
                node.terminate()
            for _, rxnode in self._sp_nodes.items():
                rxnode: RxNode
                if not rxnode.has_shutdown:
                    self.backend.logdebug(f"[{self.name}][{rxnode.name}] Shutting down.")
                    rxnode.node_shutdown()
            if not self.supervisor_node.has_shutdown:
                self.supervisor_node.node_shutdown()
            if not self.environment_node.has_shutdown:
                self.environment_node.node_shutdown()
            self.mb.shutdown()
            self.backend.delete_param(f"/{self.name}", level=1)
            self.backend.shutdown()
            self.has_shutdown = True

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
        self.supervisor.reset()
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
        self.supervisor.step()
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

    @functools.wraps(Graph.gui)
    def gui(self, *args, interactive: Optional[bool] = True, resolution: Optional[List[int]] = None, filename: Optional[str] = None, **kwargs) -> Union[None, np.ndarray]:
        """Opens a graphical user interface of the graph that was used to initialize this environment.

        .. note:: Requires `eagerx-gui`:

                .. highlight:: python
                .. code-block:: python

                    pip3 install eagerx-gui

        :param interactive: If `True`, an interactive application is launched.
                            Otherwise, an RGB render of the GUI is returned.
                            This could be useful when using a headless machine.
        :param resolution: Specifies the resolution of the returned render when `interactive` is `False`.
                           If `interactive` is `True`, this argument is ignored.
        :param filename: If provided, the GUI is rendered to an svg file with this name.
                         If `interactive` is `True`, this argument is ignored.
        :return: RGB render of the GUI if `interactive` is `False`.
        """
        return self.graph.gui(*args, interactive=interactive, resolution=resolution, filename=filename, **kwargs)

    def save(self, file: str) -> None:
        """Saves the (engine-specific) graph state, that includes the engine & environment nodes.

        The state is saved in *.yaml* format and contains the state of every added node, action, and observation
        and the connections between them.

        :param file: A string giving the name (and the file if the file isn't in the current working directory).
        """
        return self.graph.save(file)

    @classmethod
    def load(cls, name: str, file: str, backend: BackendSpec = None, force_start: bool = True):
        """Loads an environment corresponding to the graph state.

        :param name: The name of the environment. Everything related to this environment
             (parameters, topics, nodes, etc...) will be registered under namespace: "`/name`".
        :param file: A string giving the name (and the file if the file isn't in the current working directory).
        :param backend: The backend that will govern the communication for this environment.
                        Per default, the :class:`~eagerx.backends.single_process.SingleProcess` backend is used.
        :param force_start: If there already exists an environment with the same name, the existing environment is
                            first shutdown by calling the :func:`~eagerx.core.env.BaseEnv` method before initializing this
                            environment.
        """
        graph = Graph.load(file)
        graph = copy.deepcopy(graph)
        return cls(name=name, rate=None, graph=graph, engine=None, backend=backend, force_start=force_start)

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
                self.supervisor.start_render()
            elif mode == "rgb_array":
                self.supervisor.start_render()
                img = self.supervisor.get_last_image()
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
        self.supervisor.stop_render()

    def shutdown(self):
        """A method to shutdown the environment.

        - Clear the parameters on the ROS parameter under the namespace /:attr:`~eagerx.core.env.BaseEnv.name`.

        - Close nodes (i.e. release resources and perform :class:`~eagerx.core.entities.Node.close` procedure).

        - Unregister topics that supplied the I/O communication between nodes.
        """
        self._shutdown()

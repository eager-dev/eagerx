# ROS packages required
import rospy
import rosparam
import rosgraph
from std_msgs.msg import UInt64

# EAGERX
from eagerx.core.specs import NodeSpec, ObjectSpec, BridgeSpec
from eagerx.core.entities import Node
from eagerx.core.graph import Graph
from eagerx.utils.node_utils import (
    initialize_nodes,
    wait_for_node_initialization,
    substitute_args,
)
from eagerx.core.executable_node import RxNode
from eagerx.core.executable_bridge import RxBridge
from eagerx.core.supervisor import Supervisor, SupervisorNode
from eagerx.core.rx_message_broker import RxMessageBroker
from eagerx.core.constants import process

# OTHER IMPORTS
import atexit
import abc
import cv2
import numpy as np
from copy import deepcopy
from typing import List, Union, Dict, Tuple, Callable, Optional
import gym
import logging


class Env(gym.Env):
    @staticmethod
    def create_supervisor():
        entity_type = f"{SupervisorNode.__module__}/{SupervisorNode.__name__}"
        supervisor = Node.pre_make("N/a", entity_type)
        supervisor.add_output("step", msg_type=UInt64)

        supervisor.config.name = "env/supervisor"
        supervisor.config.color = "yellow"
        supervisor.config.process = process.ENVIRONMENT
        supervisor.config.outputs = ["step"]
        return supervisor

    def __init__(self, name: str, rate: float, graph: Graph, bridge: BridgeSpec) -> None:
        assert "/" not in name, 'Environment name "%s" cannot contain the reserved character "/".' % name
        self.name = name
        self.ns = "/" + name
        self.rate = rate
        self.initialized = False
        self.has_shutdown = False

        # Take deepcopy of bridge
        bridge = BridgeSpec(bridge.params)
        self._bridge_name = bridge.params["config"]["entity_id"]

        # Register graph
        self.graph = graph
        nodes, objects, actions, observations, self.render_node = graph.register()

        # Add bridge implementation
        [o.add_bridge(self._bridge_name) for o in objects]

        # Initialize supervisor node
        self.mb, self.supervisor_node, self.supervisor = self._init_supervisor(bridge, nodes, objects)
        self._is_initialized = self.supervisor_node.is_initialized

        # Initialize bridge
        self._init_bridge(bridge, nodes)

        # Create environment node
        self.env_node, self.env = self._init_environment(actions, observations, self.supervisor_node, self.mb)

        # Register render node
        if self.render_node:
            nodes = [self.render_node] + nodes

        # Register nodes
        self.register_nodes(nodes)

        # Register objects
        self.register_objects(objects)

        # Implement clean up
        atexit.register(self.shutdown)

    def _init_supervisor(self, bridge: BridgeSpec, nodes: List[NodeSpec], objects: List[ObjectSpec]):
        # Initialize supervisor
        supervisor = self.create_supervisor()

        # Get all states from objects & nodes
        for i in [bridge] + nodes + objects:
            if "states" not in i.params["config"]:
                continue
            for cname in i.params["config"]["states"]:
                entity_name = i.config.name
                name = f"{entity_name}/{cname}"
                address = f"{entity_name}/states/{cname}"
                msg_type = i.params["states"][cname]["msg_type"]
                space_converter = i.params["states"][cname]["space_converter"]

                assert (
                    name not in supervisor.params["states"]
                ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                mapping = dict(address=address, msg_type=msg_type, converter=space_converter)
                with supervisor.states as d:
                    d[name] = mapping
                supervisor.config.states.append(name)

            # Get states from simnodes. WARNING: can make environment non-agnostic.
            if isinstance(i, ObjectSpec):
                obj_name = i.config.name
                context = {"ns": {"obj_name": obj_name}, "config": i.config.to_dict()}
                for node_name, params_simnode in i.params[self._bridge_name]["nodes"].items():
                    if "states" in params_simnode["config"]:
                        for cname in params_simnode["config"]["states"]:
                            comp_params = params_simnode["states"][cname]
                            node_name_sub = substitute_args(node_name, context=context, only=["ns", "config"])
                            name = f"{node_name_sub}/{cname}"
                            address = f"{node_name_sub}/states/{cname}"
                            msg_type = comp_params["msg_type"]
                            space_converter = comp_params["space_converter"]

                            rospy.logwarn(
                                f'Adding state "{name}" to simulation node "{node_name_sub}" can potentially make the agnostic environment with object "{entity_name}" engine-specific. Check the spec of "{i.config.entity_id}" under bridge implementation "{self._bridge_name}" for more info.'
                            )
                            assert (
                                name not in supervisor.params["states"]
                            ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                            mapping = dict(
                                address=address,
                                msg_type=msg_type,
                                converter=space_converter,
                            )
                            with supervisor.states as d:
                                d[name] = mapping
                            supervisor.config.states.append(name)

        # Delete pre-existing parameters
        try:
            rosparam.delete_param(f"/{self.name}")
            rospy.loginfo(f'Pre-existing parameters under namespace "/{self.name}" deleted.')
        except rosgraph.masterapi.Error:
            pass

        # Upload log_level
        log_level = logging.getLogger("rosout").getEffectiveLevel()
        rosparam.upload_params(self.ns, {"log_level": log_level})

        # Initialize message broker
        mb = RxMessageBroker(owner="%s/%s" % (self.ns, "env"))

        # Get info from bridge on reactive properties
        is_reactive = bridge.config.is_reactive
        real_time_factor = bridge.config.real_time_factor
        simulate_delays = bridge.config.simulate_delays

        # Create supervisor node
        name = supervisor.config.name
        supervisor.config.rate = self.rate
        supervisor_params = supervisor.build(ns=self.ns)
        rosparam.upload_params(self.ns, supervisor_params)
        rx_supervisor = Supervisor(
            "%s/%s" % (self.ns, name),
            mb,
            is_reactive,
            real_time_factor,
            simulate_delays,
        )
        rx_supervisor.node_initialized()

        # Connect io
        mb.connect_io()
        return mb, rx_supervisor.node, rx_supervisor

    def _init_bridge(self, bridge: BridgeSpec, nodes: List[NodeSpec]) -> None:
        # Check that reserved keywords are not already defined.
        assert (
            "node_names" not in bridge.params["config"]
        ), f'Keyword "{"node_names"}" is a reserved keyword within the bridge params and cannot be used twice.'
        assert (
            "target_addresses" not in bridge.params["config"]
        ), f'Keyword "{"target_addresses"}" is a reserved keyword within the bridge params and cannot be used twice.'
        assert (
            not bridge.params["config"]["process"] == process.BRIDGE
        ), "Cannot initialize the bridge inside the bridge process, because it has not been launched yet. You can choose process.{ENVIRONMENT, EXTERNAL, NEW_PROCESS}."

        # Extract node_names
        node_names = ["environment", "env/supervisor"]
        target_addresses = []
        for i in nodes:
            # node_names.append(i.params['default']['name'])
            if "targets" in i.params["config"]:
                for cname in i.params["config"]["targets"]:
                    address = i.params["targets"][cname]["address"]
                    target_addresses.append(address)
        with bridge.config as d:
            d.node_names = node_names
            d.target_addresses = target_addresses

        initialize_nodes(
            bridge,
            process.ENVIRONMENT,
            self.ns,
            self.mb,
            self.supervisor_node.is_initialized,
            self.supervisor_node.sp_nodes,
            self.supervisor_node.launch_nodes,
            rxnode_cls=RxBridge,
        )
        wait_for_node_initialization(self._is_initialized)  # Proceed after bridge is initialized

    def _init_environment(self, actions: NodeSpec, observations: NodeSpec, supervisor_node, message_broker):
        # Check that env has at least one input & output.
        assert (
            len(observations.params["config"]["inputs"]) > 0
        ), f'Environment "{self.name}" must have at least one input (i.e. input).'
        assert (
            len(actions.params["config"]["outputs"]) > 0
        ), f'Environment "{self.name}" must have at least one action (i.e. output).'

        # Check that all observation addresses are unique
        addresses_obs = [observations.params["inputs"][cname]["address"] for cname in observations.params["config"]["inputs"]]
        len(set(addresses_obs)) == len(
            addresses_obs
        ), "Duplicate observations found: %s. Make sure to only have unique observations" % (
            set([x for x in addresses_obs if addresses_obs.count(x) > 1])
        )

        # Create env node
        env_spec = Node.make("Environment", rate=self.rate)
        name = env_spec.config.name
        inputs = observations.config.inputs
        outputs = actions.config.outputs
        for i in inputs:
            if i == "actions_set":
                continue
            with env_spec.inputs as d:
                d[i] = getattr(observations.inputs, i)
            env_spec.config.inputs.append(i)
        for i in outputs:
            if i == "set":
                continue
            with env_spec.outputs as d:
                d[i] = getattr(actions.outputs, i)
                d[i].rate = self.rate
            env_spec.config.outputs.append(i)
        env_params = env_spec.build(ns=self.ns)
        rosparam.upload_params(self.ns, env_params)
        rx_env = RxNode(name="%s/%s" % (self.ns, name), message_broker=message_broker)
        rx_env.node_initialized()

        # Set env_node in supervisor
        supervisor_node.set_environment(rx_env.node)

        return rx_env.node, rx_env

    @property
    def observation_space(self) -> gym.spaces.Dict:
        """Infers the observation space from the :class:`~eagerx.core.entities.SpaceConverter` of every observation.

        This space defines the format of valid observations.

        .. note:: Observations with :attr:`~eagerx.core.specs.RxInput.window` = 0 are excluded from the observation space.
                  For observations with :attr:`~eagerx.core.specs.RxInput.window` > 1,
                  the observation space is duplicated :attr:`~window` times.

        :returns: A dictionary with *key* = *observation* and *value* = :class:`Space` obtained with
                  :func:`~eagerx.core.entities.SpaceConverter.get_space`.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        observation_space = dict()
        for name, buffer in self.env_node.observation_buffer.items():
            space = buffer["converter"].get_space()
            if not buffer["window"] > 0:
                continue
            low = np.repeat(space.low[np.newaxis, ...], buffer["window"], axis=0)
            high = np.repeat(space.high[np.newaxis, ...], buffer["window"], axis=0)
            stacked_space = gym.spaces.Box(low=low, high=high, dtype=space.dtype)
            observation_space[name] = stacked_space
        return gym.spaces.Dict(spaces=observation_space)

    @property
    def action_space(self) -> gym.spaces.Dict:
        """Infers the action space from the :class:`~eagerx.core.entities.SpaceConverter` of every action.

        This space defines the format of valid actions.

        :returns: A dictionary with *key* = *action* and *value* = :class:`Space`
                  obtained with :func:`~eagerx.core.entities.SpaceConverter.get_space`.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        action_space = dict()
        for name, buffer in self.env_node.action_buffer.items():
            action_space[name] = buffer["converter"].get_space()
        return gym.spaces.Dict(spaces=action_space)

    @property
    def state_space(self) -> gym.spaces.Dict:
        """Infers the state space from the :class:`~eagerx.core.entities.SpaceConverter` of every state.

        This space defines the format of valid states that can be set before the start of an episode.

        :returns: A dictionary with *key* = *state* and *value* = :class:`Space`
                  obtained with :func:`~eagerx.core.entities.SpaceConverter.get_space`.
        """
        state_space = dict()
        for name, buffer in self.supervisor_node.state_buffer.items():
            state_space[name] = buffer["converter"].get_space()
        return gym.spaces.Dict(spaces=state_space)

    def _set_action(self, action) -> None:
        # Set actions in buffer
        for name, buffer in self.env_node.action_buffer.items():
            assert not self.supervisor_node.is_reactive or name in action, (
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
        wait_for_node_initialization(self._is_initialized)

        # Initialize single process communication
        self.mb.connect_io(print_status=True)

        rospy.loginfo("Nodes initialized.")

        # Perform first reset
        self.supervisor_node.reset()

        # Nodes initialized
        self.initialized = True
        rospy.loginfo("Pipelines initialized.")

    def _reset(self, states: Dict) -> Dict:
        """A private method that should be called within :func:`~eagerx.core.env.EagerxEnv.reset()`.

        :param states: The desired states to be set before the start an episode.
                       May also be an empty dict if no states need to be reset.
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

    def _step(self, action: Dict) -> Dict:
        """A private method that should be called within :func:`~eagerx.core.env.EagerxEnv.step()`.

        :param action: The actions to be applied in the next timestep.
                       Should include all registered actions.
        :returns: The observation of the current timestep.
        """
        # Check that nodes were previously initialized.
        assert self.initialized, "Not yet initialized. Call .reset() before calling .step()."
        assert not self.has_shutdown, "This environment has been shutdown."

        # Set actions in buffer
        self._set_action(action)

        # Call step
        self.supervisor_node.step()
        return self._get_observation()

    def _shutdown(self):
        if not self.has_shutdown:
            for address, node in self.supervisor_node.launch_nodes.items():
                rospy.loginfo(f"[{self.name}] Send termination signal to '{address}'.")
                node.terminate()
                # node.terminate(f"[{self.name}] Terminating '{address}'")
            for _, rxnode in self.supervisor_node.sp_nodes.items():
                rxnode: RxNode
                if not rxnode.has_shutdown:
                    rospy.loginfo(f"[{self.name}][{rxnode.name}] Shutting down.")
                    rxnode.node_shutdown()
            if not self.supervisor.has_shutdown:
                self.supervisor.node_shutdown()
            if not self.env.has_shutdown:
                self.env.node_shutdown()
            self.mb.shutdown()
            try:
                rosparam.delete_param(f"/{self.name}")
                rospy.loginfo(f'Parameters under namespace "/{self.name}" deleted.')
            except rosgraph.masterapi.ROSMasterException as e:
                rospy.logwarn(e)
            self.has_shutdown = True

    def register_nodes(self, nodes: Union[List[NodeSpec], NodeSpec]) -> None:
        assert not self.has_shutdown, "This environment has been shutdown."
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        if not isinstance(nodes, list):
            nodes = [nodes]

        # Register nodes
        [self.supervisor_node.register_node(n) for n in nodes]

    def register_objects(self, objects: Union[List[ObjectSpec], ObjectSpec]) -> None:
        assert not self.has_shutdown, "This environment has been shutdown."
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        if not isinstance(objects, list):
            objects = [objects]

        # Register objects
        [self.supervisor_node.register_object(o, self._bridge_name) for o in objects]

    def render(self, mode: str = "human") -> Optional[np.ndarray]:
        """A method to start rendering (i.e. open the render window).

        A message of type :class:`std_msgs.msg.Bool` is sent to topic address
        ":attr:`~eagerx.core.env.EagerxEnv.name` */env/render/toggle*", which toggles the rendering on/off.

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
                ros_im = self.supervisor_node.get_last_image()
                if ros_im.height == 0 or ros_im.width == 0:
                    # todo: check if channel dim first or last.
                    im = np.empty(shape=(0, 0, 3), dtype=np.uint8)
                else:
                    im = np.frombuffer(ros_im.data, dtype=np.uint8).reshape(ros_im.height, ros_im.width, -1)
                    if "bgr" in ros_im.encoding:
                        # try:
                        # todo: find out what exception to catch here.
                        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
                        # except :
                        #     pass
                return im
            else:
                raise ValueError('Render mode "%s" not recognized.' % mode)
        else:
            rospy.logwarn_once("No render node active, so not rendering.")
            if mode == "rgb_array":
                return np.empty((0, 0, 3), dtype="uint8")
            else:
                return

    @abc.abstractmethod
    def reset(self) -> Dict:
        """An abstract method that resets the environment to an initial state and returns an initial observation.

        :returns: The initial observation.
        """
        pass

    @abc.abstractmethod
    def step(self, action: Dict) -> Tuple[Dict, float, bool, Dict]:
        """An abstract method that runs one timestep of the environment's dynamics.

        When the end of an episode is reached, you are responsible for calling :func:`~eagerx.core.Env.reset`
        to reset this environment's state.

        :params action: A dictionary of actions provided by the agent. Should include all registered actions.

        :returns: A tuple (observation, reward, done, info).

                  - observation: Dictionary of observations of the current timestep.

                  - reward: amount of reward returned after previous action

                  - done: whether the episode has ended, in which case further step() calls will return undefined results

                  - info: contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        pass

    def close(self):
        """A method to stop rendering (i.e. close the render window).

        A message of type :class:`std_msgs.msg.Bool` is sent to topic address
        ":attr:`~eagerx.core.env.EagerxEnv.name` */env/render/toggle*", which toggles the rendering on/off.

        .. note:: Depending on the source node that is producing the images that are rendered,
                  images may still be produced, even when the render window is not visible.
                  This may add computational overhead and influence the run speed.

                  Optionally, users may subscribe to topic address ":attr:`~eagerx.core.env.EagerxEnv.name` */env/render/toggle*"
                  in the node that is producing the images to stop the production and output empty
                  :class:`sensor_msgs.msg.Image` messages instead.
        """
        assert not self.has_shutdown, "This environment has been shutdown."
        self.supervisor_node.stop_render()

    def shutdown(self):
        """A method to shutdown the environment.

        - Clear the parameters on the ROS parameter under the namespace /:attr:`~eagerx.core.env.EagerxEnv.name`.

        - Close nodes (i.e. release resources and perform :class:`~eagerx.core.entities.Node.close` procedure).

        - Unregister topics that supplied the I/O communication between nodes.
        """
        self._shutdown()


class EagerxEnv(Env):
    """The main EAGERx environment class that follows the OpenAI gym's Env API.

    Users can directly use this class, but may also choose to subclass it and inline the environment construction
    (e.g. node creation, graph connecting, bridge selection, etc...) into :func:`~eagerx.core.env.EagerxEnv.__init__`.

     A subclass may implement/overwrite the following methods:

    - :func:`~eagerx.core.env.EagerxEnv.__init__`: Be sure to call :func:`super().__init__` inside this method with the required arguments.

    - :func:`~eagerx.core.env.EagerxEnv.step`: Be sure to call :func:`~eagerx.core.env.EagerxEnv._step` inside this method to perform the step.

    - :func:`~eagerx.core.env.EagerxEnv.reset`: Be sure to call :func:`~eagerx.core.env.EagerxEnv._reset` inside this method to perform the reset.
    """

    def __init__(
        self,
        name: str,
        rate: float,
        graph: Graph,
        bridge: BridgeSpec,
        step_fn: Callable = lambda prev_obs, obs, action, steps: (obs, 0.0, False, {}),
        reset_fn: Callable = lambda env: env.state_space.sample(),  # noqa: B008
        exclude: Optional[List[str]] = None,
    ) -> None:
        """Initializes an environment with EAGERx dynamics.

        :param name: The name of the environment. Everything related to this environment
                     (parameters, topics, nodes, etc...) will be registered under namespace: "`/name`".
        :param rate: The rate (Hz) at which the environment will run.
        :param graph: The graph consisting of nodes and objects that describe the environment's dynamics.
        :param bridge: The physics engine that will govern the environment's dynamics.
                       For every :class:`~eagerx.core.entities.Object` in the graph,
                       the corresponding bridge implementations is chosen.
        :param step_fn: A callable that provides the tuple (observation, reward, done, info) after the environment has run one timestep.
                        As arguments, the provided callable receives the previous observation, current observation, applied action,
                        and number of timesteps since the last reset.
        :param reset_fn: A callable that returns a dictionary with the desired states to be set before the start an episode.
                         Valid states are described by :attr:`~eagerx.core.env.EagerxEnv.state_space`.
        :param exclude: Key names of the observations that are excluded from the observation space. In other words,
                        the observations that will not be returned as part of the dict when the agent calls
                        :func:`~eagerx.core.EagerxEnv.reset` and :func:`~eagerx.core.EagerxEnv.step`.
                        Observations with `window=0` are already excluded.
        """
        self.steps = None
        self.prev_observation = None
        #: A callable that provides the tuple (observation, reward, done, info) after the environment has run one timestep.
        #: As arguments, the provided callable receives the previous observation, current observation, applied action,
        #: and number of timesteps since the last reset.
        self.step_fn = step_fn
        #: A callable that returns a dictionary with the desired states to be set before the start an episode.
        #: Valid states are described by :attr:`~eagerx.core.env.EagerxEnv.state_space`.
        #: May also be an empty dictionary if no states need to be reset.
        self.reset_fn = reset_fn
        super(EagerxEnv, self).__init__(name, rate, graph, bridge)

        # Determine set of observations to exclude
        exclude = exclude if isinstance(exclude, list) else []
        zero_window = [name for name, buffer in self.env_node.observation_buffer.items() if buffer["window"] == 0]
        self.excl_nonzero = [name for name in exclude if name not in zero_window]
        self.excl_obs = exclude + [name for name in zero_window if name not in exclude]

        # Check if all excluded observations with window > 0 actually exist
        space = super(EagerxEnv, self).observation_space
        nonexistent = [name for name in self.excl_nonzero if name not in space.spaces]
        if len(nonexistent) != 0:
            rospy.logwarn(f"Some excluded observations with window > 0 do not exist: {nonexistent}.")

    @property
    def observation_space(self):
        """Infers the observation space from the :class:`~eagerx.core.entities.SpaceConverter` of every observation.

        This space defines the format of valid observations.

        .. note:: Observations specified in the `exclude` argument in :func:`~eagerx.core.EagerxEnv.__init__` are excluded.
                  Observations with :attr:`~eagerx.core.specs.RxInput.window` = 0 are also excluded from the observation space.
                  For observations with :attr:`~eagerx.core.specs.RxInput.window` > 1,
                  the observation space is duplicated :attr:`~window` times.

        :returns: A dictionary with *key* = *observation* and *value* = :class:`Space` obtained with
                  :func:`~eagerx.core.entities.SpaceConverter.get_space`.
        """
        if len(self.excl_nonzero) > 0:
            obs_space = super(EagerxEnv, self).observation_space.spaces
            [obs_space.pop(name) for name in self.excl_nonzero]
            return gym.spaces.Dict(obs_space)
        else:
            return super(EagerxEnv, self).observation_space

    def step(self, action: Dict) -> Tuple[Dict, float, bool, Dict]:
        """A method that runs one timestep of the environment's dynamics.

        When the end of an episode is reached, you are responsible for calling :func:`~eagerx.core.EagerxEnv.reset`
        to reset this environment's state.

        After the action is applied and the environment's dynamics have run one timestep,
        :attr:`~eagerx.core.env.EagerxEnv.step_fn` is called that returns the tuple (observation, reward, done, info).

        .. note:: Observations with :attr:`~eagerx.core.specs.RxInput.window` = 0 are excluded from the observation
                  dictionary returned to the agent.
                  However, they are nonetheless available in :attr:`~eagerx.core.env.EagerxEnv.step_fn` to,
                  for example, calculate the reward or the episode termination condition.

        :params action: A dictionary of actions provided by the agent.
        :returns: A tuple (observation, reward, done, info).

                  - observation: Dictionary of observations of the current timestep.

                  - reward: amount of reward returned after previous action

                  - done: whether the episode has ended, in which case further step() calls will return undefined results

                  - info: contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        # Send actions and wait for observations (i.e. apply step)
        observation = self._step(action)
        self.steps += 1

        # Save observation for next step
        prev_obs = deepcopy(observation)

        # Process (e.g. calculate reward) after applying the action
        observation, reward, is_done, info = self.step_fn(self.prev_observation, observation, action, self.steps)

        # Pop all excluded observations and the ones with window = 0 (if present)
        for name in self.excl_obs:
            observation.pop(name, None)

        # Update previous observation with current observation (used in next step)
        self.prev_observation = prev_obs
        return observation, reward, is_done, info

    def reset(self) -> Dict:
        """Resets the environment to an initial state and returns an initial
        observation.

        The initial state is set with the return value of :attr:`~eagerx.core.env.EagerxEnv.reset_fn`.

        :returns: The initial observation.
        """
        # Determine reset states
        states = self.reset_fn(self)

        # Perform reset
        observation = self._reset(states)
        self.prev_observation = deepcopy(observation)

        # Pop all excluded observations and the ones with window = 0 (if present)
        for name in self.excl_obs:
            observation.pop(name, None)

        # Reset number of steps
        self.steps = 0
        return observation

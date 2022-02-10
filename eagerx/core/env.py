# ROS packages required
import rospy
import rosparam
from std_msgs.msg import UInt64
from rosgraph.masterapi import Error
from sensor_msgs.msg import Image

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
import abc
import numpy as np
from copy import deepcopy
from typing import List, Union, Dict, Tuple, Callable
import gym
import logging


class Env(gym.Env):
    @staticmethod
    def create_supervisor():
        entity_type = f"{SupervisorNode.__module__}/{SupervisorNode.__name__}"
        supervisor = Node.pre_make("N/a", entity_type)
        supervisor.add_output("step", msg_type=UInt64)
        supervisor.set_parameter("name", "env/supervisor")
        supervisor.set_parameter("color", "yellow")
        supervisor.set_parameter("process", process.ENVIRONMENT)
        supervisor.set_parameter("outputs", ["step"])
        return supervisor

    def __init__(self, name: str, rate: float, graph: Graph, bridge: BridgeSpec) -> None:
        assert "/" not in name, 'Environment name "%s" cannot contain the reserved character "/".' % name
        self.name = name
        self.ns = "/" + name
        self.rate = rate
        self.initialized = False
        self._bridge_name = bridge.params["default"]["entity_id"]

        # Register graph
        self.graph = graph
        nodes, objects, actions, observations, self.render_node = graph.register()

        # Initialize supervisor node
        self.mb, self.supervisor_node, _ = self._init_supervisor(bridge, nodes, objects)
        self._is_initialized = self.supervisor_node.is_initialized

        # Initialize bridge
        self._init_bridge(bridge, nodes)

        # Create environment node
        self.env_node, _ = self._init_environment(actions, observations, self.supervisor_node, self.mb)

        # Register render node
        if self.render_node:
            nodes = [self.render_node] + nodes

        # Register nodes
        self.register_nodes(nodes)

        # Register objects
        self.register_objects(objects)

    def _init_supervisor(self, bridge: BridgeSpec, nodes: List[NodeSpec], objects: List[ObjectSpec]):
        # Initialize supervisor
        supervisor = self.create_supervisor()

        # Get all states from objects & nodes
        for i in [bridge] + nodes + objects:
            if "states" not in i.params["default"]:
                continue
            for cname in i.params["default"]["states"]:
                entity_name = i.get_parameter("name")
                name = f"{entity_name}/{cname}"
                address = f"{entity_name}/states/{cname}"
                msg_type = i.params["states"][cname]["msg_type"]
                space_converter = i.params["states"][cname]["space_converter"]

                assert (
                    name not in supervisor.params["states"]
                ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                mapping = dict(address=address, msg_type=msg_type, converter=space_converter)
                supervisor._set({"states": {name: mapping}})
                supervisor._params["default"]["states"].append(name)

            # Get states from simnodes. WARNING: can make environment non-agnostic.
            if isinstance(i, ObjectSpec):
                obj_name = i.get_parameter("name")
                context = {"ns": {"obj_name": obj_name}, "default": i.get_parameters()}
                for node_name, params_simnode in i.params[self._bridge_name]["nodes"].items():
                    if "states" in params_simnode["default"]:
                        for cname in params_simnode["default"]["states"]:
                            comp_params = params_simnode["states"][cname]
                            node_name_sub = substitute_args(node_name, context=context, only=["ns", "default"])
                            name = f"{node_name_sub}/{cname}"
                            address = f"{node_name_sub}/states/{cname}"
                            msg_type = comp_params["msg_type"]
                            space_converter = comp_params["space_converter"]

                            rospy.logwarn(
                                f'Adding state "{name}" to simulation node "{node_name_sub}" can potentially make the agnostic environment with object "{entity_name}" engine-specific. Check the spec of "{i.get_parameter("entity_id")}" under bridge implementation "{self._bridge_name}" for more info.'
                            )
                            assert (
                                name not in supervisor.params["states"]
                            ), f'Cannot have duplicate states. State "{name}" is defined multiple times.'

                            mapping = dict(
                                address=address,
                                msg_type=msg_type,
                                converter=space_converter,
                            )
                            supervisor._set({"states": {name: mapping}})
                            supervisor._params["default"]["states"].append(name)

        # Delete pre-existing parameters
        try:
            rosparam.delete_param(f"/{self.name}")
            rospy.loginfo(f'Pre-existing parameters under namespace "/{self.name}" deleted.')
        except Error:
            pass

        # Upload log_level
        log_level = logging.getLogger("rosout").getEffectiveLevel()
        rosparam.upload_params(self.ns, {"log_level": log_level})

        # Initialize message broker
        mb = RxMessageBroker(owner="%s/%s" % (self.ns, "env"))

        # Get info from bridge on reactive properties
        is_reactive = bridge.get_parameter("is_reactive")
        real_time_factor = bridge.get_parameter("real_time_factor")
        simulate_delays = bridge.get_parameter("simulate_delays")

        # Create supervisor node
        name = supervisor.get_parameter("name")
        supervisor.set_parameter("rate", self.rate)
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
            "node_names" not in bridge.params["default"]
        ), f'Keyword "{"node_names"}" is a reserved keyword within the bridge params and cannot be used twice.'
        assert (
            "target_addresses" not in bridge.params["default"]
        ), f'Keyword "{"target_addresses"}" is a reserved keyword within the bridge params and cannot be used twice.'
        assert (
            not bridge.params["default"]["process"] == process.BRIDGE
        ), "Cannot initialize the bridge inside the bridge process, because it has not been launched yet. You can choose process.{ENVIRONMENT, EXTERNAL, NEW_PROCESS}."

        # Extract node_names
        node_names = ["environment", "env/supervisor"]
        target_addresses = []
        for i in nodes:
            # node_names.append(i.params['default']['name'])
            if "targets" in i.params["default"]:
                for cname in i.params["default"]["targets"]:
                    address = i.params["targets"][cname]["address"]
                    target_addresses.append(address)
        bridge._set({"default": {"node_names": node_names}})
        bridge._set({"default": {"target_addresses": target_addresses}})

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
            len(observations.params["default"]["inputs"]) > 0
        ), f'Environment "{self.name}" must have at least one input (i.e. input).'
        assert (
            len(actions.params["default"]["outputs"]) > 0
        ), f'Environment "{self.name}" must have at least one action (i.e. output).'

        # Check that all observation addresses are unique
        addresses_obs = [
            observations.params["inputs"][cname]["address"] for cname in observations.params["default"]["inputs"]
        ]
        len(set(addresses_obs)) == len(
            addresses_obs
        ), "Duplicate observations found: %s. Make sure to only have unique observations" % (
            set([x for x in addresses_obs if addresses_obs.count(x) > 1])
        )

        # Create env node
        env_spec = Node.make("Environment", rate=self.rate)
        name = env_spec.get_parameter("name")
        inputs = observations.get_parameter("inputs")
        outputs = actions.get_parameter("outputs")
        for i in inputs:
            if i == "actions_set":
                continue
            env_spec._params["inputs"][i] = observations.get_parameters("inputs", i)
            env_spec._params["default"]["inputs"].append(i)
        for i in outputs:
            if i == "set":
                continue
            env_spec._params["outputs"][i] = actions.get_parameters("outputs", i)
            env_spec._params["default"]["outputs"].append(i)
        env_params = env_spec.build(ns=self.ns)
        rosparam.upload_params(self.ns, env_params)
        rx_env = RxNode(name="%s/%s" % (self.ns, name), message_broker=message_broker)
        rx_env.node_initialized()

        # Set env_node in supervisor
        supervisor_node.set_environment(rx_env.node)

        return rx_env.node, rx_env

    @property
    def observation_space(self) -> gym.spaces.Dict:
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
        action_space = dict()
        for name, buffer in self.env_node.action_buffer.items():
            action_space[name] = buffer["converter"].get_space()
        return gym.spaces.Dict(spaces=action_space)

    @property
    def state_space(self) -> gym.spaces.Dict:
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

    def _initialize(self) -> None:
        assert not self.initialized, "Environment already initialized. Cannot re-initialize pipelines. "

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
        # Initialize environment
        if not self.initialized:
            self._initialize()

        # Set desired reset states
        self._set_state(states)

        # Perform reset
        self.supervisor_node.reset()
        obs = self._get_observation()
        return obs

    def _step(self, action: Dict) -> Dict:
        # Check that nodes were previously initialized.
        assert self.initialized, "Not yet initialized. Call .reset() before calling .step()."

        # Set actions in buffer
        self._set_action(action)

        # Call step
        self.supervisor_node.step()
        return self._get_observation()

    def _shutdown(self):
        for name in self.supervisor_node.launch_nodes:
            self.supervisor_node.launch_nodes[name].shutdown()
        try:
            rosparam.delete_param("/")
            rospy.loginfo('Pre-existing parameters under namespace "/" deleted.')
        except:
            pass

    def register_nodes(self, nodes: Union[List[NodeSpec], NodeSpec]) -> None:
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        if not isinstance(nodes, list):
            nodes = [nodes]

        # Register nodes
        [self.supervisor_node.register_node(n) for n in nodes]

    def register_objects(self, objects: Union[List[ObjectSpec], ObjectSpec]) -> None:
        # Look-up via <env_name>/<obj_name>/nodes/<component_type>/<component>: /rx/obj/nodes/sensors/pos_sensors
        if not isinstance(objects, list):
            objects = [objects]

        # Register objects
        [self.supervisor_node.register_object(o, self._bridge_name) for o in objects]

    def render(self, mode="human"):
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
                return im
            else:
                raise ValueError('Render mode "%s" not recognized.' % mode)
        else:
            rospy.logwarn_once("No render node active, so not rendering.")
            if mode == "rgb_array":
                return Image()
            else:
                return

    @abc.abstractmethod
    def reset(self) -> Dict:
        pass

    @abc.abstractmethod
    def step(self, action: Dict) -> Tuple[Dict, float, bool, Dict]:
        pass

    def close(self):
        self.supervisor_node.stop_render()

    def shutdown(self):
        self._shutdown()


class EagerEnv(Env):
    def __init__(
        self,
        name: str,
        rate: float,
        graph: Graph,
        bridge: BridgeSpec,
        step_fn: Callable = lambda prev_obs, obs, action, steps: (obs, 0.0, False, {}),
        reset_fn: Callable = lambda env: env.state_space.sample(),
    ) -> None:
        self.steps = None
        self.prev_observation = None
        self.step_fn = step_fn
        self.reset_fn = reset_fn
        super(EagerEnv, self).__init__(name, rate, graph, bridge)
        self.excl_obs = [name for name, buffer in self.env_node.observation_buffer.items() if buffer["window"] == 0]

    def step(self, action: Dict) -> Tuple[Dict, float, bool, Dict]:
        # Send actions and wait for observations
        observation = self._step(action)
        self.steps += 1

        # Save observation for next step
        prev_obs = deepcopy(observation)

        # Process after step
        observation, reward, is_done, info = self.step_fn(self.prev_observation, observation, action, self.steps)

        # Pop all observations with window = 0 (if present)
        for name in self.excl_obs:
            try:
                observation.pop(name)
            except KeyError:
                pass

        # Update previous observation with current observation (used in next step)
        self.prev_observation = prev_obs
        return observation, reward, is_done, info

    def reset(self) -> Dict:
        # Determine reset states
        states = self.reset_fn(self)

        # Perform reset
        observation = self._reset(states)
        self.prev_observation = deepcopy(observation)

        # Reset number of steps
        self.steps = 0
        return observation

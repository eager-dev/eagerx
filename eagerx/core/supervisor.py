# Rx imports
import eagerx.utils.utils
from eagerx.core.constants import process
from eagerx.core.executable_node import RxNode
import eagerx.core.rx_message_broker
import eagerx.core.rx_operators
import eagerx.core.rx_pipelines
from eagerx.core.entities import BaseNode
from eagerx.core.specs import NodeSpec, ObjectSpec
from eagerx.utils.utils import (
    load,
    initialize_processor,
    get_param_with_blocking,
)
from eagerx.utils.node_utils import initialize_nodes
from eagerx.core.nodes import EnvNode
import eagerx

# OTHER
from threading import Event


class SupervisorNode(BaseNode):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.subjects = None
        self.env_node: EnvNode = None

        # Render
        self.last_image = None
        self._image_event = Event()
        self.render_toggle = False
        self.pub_get_last_image = self.backend.Publisher(f"{self.ns}/env/render/get_last_image", "bool")
        self.sub_set_last_image = self.backend.Subscriber(
            f"{self.ns}/env/render/set_last_image", "uint8", self._last_image_callback
        )
        self.render_toggle_pub = self.backend.Publisher(f"{self.ns}/env/render/toggle", "bool")

        # Initialize nodes
        self.cum_registered = 0
        self.is_initialized = dict()
        self.launch_nodes = dict()
        self.sp_nodes = dict()

        # Initialize buffer to hold desired reset states
        self.state_buffer = dict()
        for cname, i in self.states.items():
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])
            assert i["space"] is not None, f"No space defined for state {cname}."
            assert i["space"].is_fully_defined, f"The space for state {cname} is not fully defined (low, high, shape, dtype)."
            self.state_buffer[cname] = {"msg": None, "processor": i["processor"], "space": i["space"]}

        # Required for reset
        self._step_counter = 0

    def set_environment(self, env_node: EnvNode):
        self.env_node = env_node

    def _set_subjects(self, subjects):
        self.subjects = subjects

    def start_render(self):
        if not self.render_toggle:
            self.render_toggle = True
            self.render_toggle_pub.publish(self.render_toggle)

    def stop_render(self):
        if self.render_toggle:
            self.render_toggle = False
            self.render_toggle_pub.publish(self.render_toggle)

    def get_last_image(self):
        self._image_event.clear()
        self.pub_get_last_image.publish(True)
        self._image_event.wait()
        return self.last_image

    def _last_image_callback(self, msg):
        self.last_image = msg
        self._image_event.set()

    def register_node(self, node: NodeSpec):
        # Increase cumulative registered counter. Is send as '/start_reset' message.
        self.cum_registered += 1

        # Initialize node
        node_name = node.config.name
        initialize_nodes(
            node,
            process.ENVIRONMENT,
            self.ns,
            self.message_broker,
            self.is_initialized,
            self.sp_nodes,
            self.launch_nodes,
            rxnode_cls=RxNode,
        )
        self.subjects["register_node"].on_next(self.ns + "/" + node_name)

    def register_object(self, object: ObjectSpec, engine_name: str):
        # Increase cumulative registered counter. Is send as '/start_reset' message.
        self.cum_registered += 1

        # Check if object name is unique
        obj_name = object.config.name
        assert (
            self.backend.get_param(self.ns + "/" + obj_name + "/nodes", None) is None
        ), f'Object name "{self.ns}/{obj_name}" already exists. Object names must be unique.'

        # Upload object params to param server
        params, nodes = object.build(ns=self.ns, engine_id=engine_name)
        self.backend.upload_params(self.ns, params)

        # Set node args
        node_args = dict(
            object_name=f"{self.ns}/{obj_name}",
        )

        # Upload node parameters to ROS param server
        initialize_nodes(
            nodes,
            process.ENVIRONMENT,
            self.ns,
            message_broker=self.message_broker,
            is_initialized=self.is_initialized,
            sp_nodes=self.sp_nodes,
            launch_nodes=self.launch_nodes,
            node_args=node_args,
            object_name=obj_name,
        )
        self.subjects["register_object"].on_next(f"{self.ns}/{obj_name}")

    def _get_states(self, reset_msg):
        # Fill output_msg with buffered states
        msgs = dict()
        for name, buffer in self.state_buffer.items():
            if buffer["msg"] is None:
                msgs[name + "/done"] = True
            else:
                msgs[name + "/done"] = False
                msgs[name] = buffer["msg"]
                buffer["msg"] = None  # After sending state, set msg to None
        return msgs

    def reset(self):
        self.env_node.obs_event.clear()
        self.env_node.must_reset = True
        self.env_node.action_event.set()
        self.subjects["start_reset"].on_next(self.cum_registered)
        self._step_counter = 0
        try:
            flag = self.env_node.obs_event.wait()
            if not flag:
                raise KeyboardInterrupt
        except (KeyboardInterrupt, SystemExit):
            self.backend.logdebug("[reset] KEYBOARD INTERRUPT")
            raise
        self.backend.logdebug("FIRST OBS RECEIVED!")

    def step(self):
        self.env_node.obs_event.clear()
        self.env_node.action_event.set()
        self._step_counter += 1
        try:
            flag = self.env_node.obs_event.wait()
            if not flag:
                raise KeyboardInterrupt
        except (KeyboardInterrupt, SystemExit):
            self.backend.logdebug("[step] KEYBOARD INTERRUPT")
            raise
        self.backend.logdebug("STEP END")

    def shutdown(self):
        self.env_node.action_event.set()
        self.pub_get_last_image.unregister()
        self.sub_set_last_image.unregister()
        self.render_toggle_pub.unregister()


class Supervisor(object):
    def __init__(self, name, message_broker, sync, real_time_factor, simulate_delays):
        self.name = name
        self.ns = "/".join(name.split("/")[:2])
        self.mb = message_broker
        self.backend = message_broker.bnd
        self.initialized = False
        self.sync = sync
        self.has_shutdown = False

        # Prepare input & output topics
        outputs, states, self.node = self._prepare_io_topics(self.name, sync, real_time_factor, simulate_delays)

        # Initialize reactive pipeline
        rx_objects, env_subjects = eagerx.core.rx_pipelines.init_supervisor(
            self.ns, self.node, outputs=outputs, state_outputs=states
        )
        self.node._set_subjects(env_subjects)
        self.mb.add_rx_objects(node_name=name, node=self, **rx_objects)

    def node_initialized(self):
        # Notify env that node is initialized
        self.init_pub = self.backend.Publisher(self.name + "/initialized", "int64")
        self.init_pub.publish(0)

        if not self.initialized:
            self.backend.loginfo('Node "%s" initialized.' % self.name)
        self.initialized = True

    def _prepare_io_topics(self, name, sync, real_time_factor, simulate_delays):
        params = get_param_with_blocking(name, self.backend)

        # Prepare output topics
        for i in params["outputs"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Prepare state topics
        for i in params["states"]:
            if isinstance(i["processor"], dict):
                from eagerx.core.specs import ProcessorSpec

                i["processor"] = initialize_processor(ProcessorSpec(i["processor"]))
            if isinstance(i["space"], dict):
                i["space"] = eagerx.Space.from_dict(i["space"])

        # Convert lists to dicts
        params["outputs"] = {i["name"]: i for i in params["outputs"]}
        params["states"] = {i["name"]: i for i in params["states"]}

        # Get node
        node_cls = load(params["node_type"])
        node = node_cls(
            ns=self.ns,
            message_broker=self.mb,
            sync=sync,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            params=params,
        )

        # Convert to tuple for reactive pipeline.
        outputs = tuple([value for key, value in params["outputs"].items()])
        states = tuple([value for key, value in params["states"].items()])

        return outputs, states, node

    def _shutdown(self):
        self.backend.logdebug(f"[{self.name}] Supervisor._shutdown() called.")
        self.init_pub.unregister()

    def node_shutdown(self):
        if not self.has_shutdown:
            self.backend.logdebug(f"[{self.name}] Supervisor.node_shutdown() called.")
            self.backend.loginfo(f"[{self.name}] Shutting down.")
            self._shutdown()
            self.node.shutdown()
            self.mb.shutdown()
            self.has_shutdown = True

import rospy


# LOG LEVELS
SILENT = 0
DEBUG = 10
INFO = 20
WARN = 30
ERROR = 40
FATAL = 50
log_levels_ROS = {
    SILENT: rospy.DEBUG,
    DEBUG: rospy.DEBUG,
    INFO: rospy.INFO,
    WARN: rospy.WARN,
    ERROR: rospy.ERROR,
    FATAL: rospy.FATAL,
}


class log:
    SILENT = SILENT
    DEBUG = DEBUG
    INFO = INFO
    WARN = WARN
    ERROR = ERROR
    FATAL = FATAL


# PRINT MODES
TERMCOLOR = 1
ROS = 2

# Terminal types
TERMS = {
    "object": {"in": {"actuators"}, "out": {"sensors", "states"}},
    "node": {"in": {"inputs"}, "out": {"outputs", "states"}},
    "reset_node": {
        "in": {"feedthroughs", "inputs", "targets"},
        "out": {"outputs", "states"},
    },
}
TERMS_IN = set().union(*[TERMS[key]["in"] for key in TERMS])
TERMS_OUT = set().union(*[TERMS[key]["out"] for key in TERMS])

# Possible entries in GUI
GUI_WIDGETS = {
    "node": {
        "hide": {
            "all": [],
            "actions": ["rate"],
            "observations": ["rate"],
        },
        "items": {
            "color": [
                "black",
                "red",
                "green",
                "yellow",
                "blue",
                "magenta",
                "cyan",
                "white",
                "grey",
            ],
            "log_level": {
                "silent": 0,
                "debug": 10,
                "info": 20,
                "warn": 30,
                "error": 40,
                "fatal": 50,
            },
            "process": {"new process": 0, "environment": 1, "bridge": 2, "external": 3},
        },
        "constant": {
            "all": list(set.union(TERMS_IN, TERMS_OUT, {"name", "entity_id", "launch_file"})),
            "actions": ["process"],
            "observations": ["process"],
        },
    },
    "term": {
        "items": {
            "repeat": ["all", "empty", "window"],
        },
        "constant": {
            "all": ["msg_type"],
            "feedthroughs": ["space_converter"],
        },
        "hide": {
            "all": [],
            "states": ["converter"],
            "targets": ["space_converter"],
            "sensors": ["start_with_msg"],
            "actions": ["start_with_msg", "space_converter"],
            "observations": ["is_reactive", "rate", "space_converter"],
        },
    },
}

# Corresponding RGB values for colors
GUI_COLORS = {
    "black": [0, 0, 0],
    "red": [255, 0, 0],
    "green": [0, 128, 0],
    "yellow": [255, 255, 0],
    "blue": [0, 0, 255],
    "magenta": [255, 0, 255],
    "cyan": [0, 255, 255],
    "white": [255, 255, 255],
}

# Config files to ignore in GUI
GUI_ENTITIES_TO_IGNORE = {
    "BaseConverter",
    "SpaceConverter",
    "Converter",
    "Processor",
    "EngineNode",
    "Bridge",
    "EngineState",
}
GUI_NODE_IDS_TO_IGNORE = {
    "Observations",
    "Actions",
    "Render",
    "Supervisor",
    "Environment",
}


# PROCESS
class process:
    #: Spawn the node/bridge in a separate process.
    #: Allows parallelization, but increases communication overhead due to the (de)serialization of messages.
    NEW_PROCESS: int = 0
    #: Spawn the node/bridge in the process of the environment.
    ENVIRONMENT: int = 1
    #: Spawn a node in the process of the bridge.
    #: If an :class:`~eagerx.core.entities.EngineNode` requires direct access to the
    #: :attr:`~eagerx.core.entities.Bridge.simulator`,
    #: :attr:`~eagerx.core.entities.EngineNode.config`, and
    #: :attr:`~eagerx.core.entities.EngineNode.bridge_config`,
    #: it must be spawned in the same process as the bridge.
    BRIDGE: int = 2
    #: Spawn the node/bridge in a separate process. This process is not spawned by the environment.
    #: Instead, the user is responsible for running the executable script with the appropriate arguments.
    #: This allows nodes to run distributed.
    EXTERNAL: int = 3

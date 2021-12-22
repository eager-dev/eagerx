import rospy

# LOG LEVELS
SILENT = 0
DEBUG = 10
INFO = 20
WARN = 30
ERROR = 40
FATAL = 50
log_levels_ROS = {SILENT: rospy.DEBUG,
                  DEBUG: rospy.DEBUG,
                  INFO: rospy.INFO,
                  WARN: rospy.WARN,
                  ERROR: rospy.ERROR,
                  FATAL: rospy.FATAL}

# PRINT MODES
TERMCOLOR = 1
ROS = 2

# Terminal types
TERMS = {
    'object': {'in': {'actuators'}, 'out': {'sensors', 'states'}},
    'node': {'in': {'inputs'}, 'out': {'outputs', 'states'}},
    'reset_node': {'in': {'feedthroughs', 'inputs', 'targets'}, 'out': {'outputs', 'states'}},
}
TERMS_IN = set().union(*[TERMS[key]['in'] for key in TERMS])
TERMS_OUT = set().union(*[TERMS[key]['out'] for key in TERMS])

# Possible entries in GUI
GUI_NODE_ITEMS = {
    'color': ['black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white'],
    'log_level': {'silent': 0, 'debug': 10, 'info': 20, 'warn': 30, 'error': 40, 'fatal': 50},
    'process': {'new process': 0, 'environment': 1, 'bridge': 2, 'external': 3},
}
GUI_TERM_ITEMS = {
    'repeat': ['all', 'empty', 'window'],
}

# Corresponding RGB values for colors
GUI_COLORS = {
    'black': [0, 0, 0],
    'red': [255, 0, 0],
    'green': [0, 128, 0],
    'yellow': [255, 255, 0],
    'blue': [0, 0, 255],
    'magenta': [255, 0, 255],
    'cyan': [0, 255, 255],
    'white': [255, 255, 255],
}

# Default arguments to ignore
GUI_IGNORE_DEFAULT = {
    'actions': ['start_with_msg', 'space_converter'],
    'observations': ['rate', 'is_reactive', 'space_converter'],
}

# Constant yaml parameters
PARAMS_CONSTANT = set.union(TERMS_IN, TERMS_OUT, {'name', 'package_name', 'config_name', 'launch_file'})

# Config files to ignore in GUI
GUI_CONFIG_TO_IGNORE = {
    'eagerx_core': ['actions', 'observations', 'bridge', 'supervisor', 'render'],
}

# PROCESS
class process:
    NEW_PROCESS = 0
    ENVIRONMENT = 1
    BRIDGE = 2
    EXTERNAL = 3

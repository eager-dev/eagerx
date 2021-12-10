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
    'reset_node': {'in': {'inputs', 'targets', 'feedthroughs'}, 'out': {'outputs', 'states'}},
}
TERMS_IN = set().union(*[TERMS[key]['in'] for key in TERMS])
TERMS_OUT = set().union(*[TERMS[key]['out'] for key in TERMS])

# Possible entries in GUI
GUI_NODE_ITEMS = {
    'color': ['black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white'],
    'log_level': ['0', '10', '20', '30', '40', '50'],
}
GUI_TERM_ITEMS = {
    'repeat': ['all', 'empty', 'last'],
}

# Constant yaml parameters
PARAMS_CONSTANT = set.union(TERMS_IN, TERMS_OUT, {'name', 'package_name', 'config_name', 'launch_file'})

# PROCESS
class process:
    NEW_PROCESS = 0
    ENVIRONMENT = 1
    BRIDGE = 2
    EXTERNAL = 3

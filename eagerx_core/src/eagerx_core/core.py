from eagerx_core.params import RxObjectParams, RxNodeParams, RxBridgeParams
import rospy

# ALIAS CLASS NAMES
RxObject = RxObjectParams
RxNode = RxNodeParams
RxBridge = RxBridgeParams

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


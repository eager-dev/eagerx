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


# PROCESS
class process:
    NEW_PROCESS = 0
    ENVIRONMENT = 1
    BRIDGE = 2
    EXTERNAL = 3

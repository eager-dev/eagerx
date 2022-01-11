# ROS packages required
import rospy
from eagerx_core.core import RxBridge, RxNode, RxObject, EAGERxEnv, RxGraph
from eagerx_core.utils.node_utils import launch_roscore
from eagerx_core.constants import process
from eagerx_core.wrappers.flatten import Flatten
from eagerx_core.baseconverter import StringUInt64Converter, ImageUInt64Converter, IdentityConverter


if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.DEBUG)
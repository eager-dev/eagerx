# ROS1
try:
    import rospy
    import rosparam
    import rosgraph
    import rosservice
    from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
    import std_msgs.msg
except ImportError as e:
    raise ImportError(f"{e}. (HINT: you need to install ROS Melodic or Noetic and have it sourced.")

# OTHER
import logging
import typing
import time
import numpy as np
import atexit
from functools import wraps

# EAGERx
import eagerx
from eagerx.core.pubsub import Publisher, Subscriber, ShutdownService
from eagerx.core.constants import (
    SILENT,
    DEBUG,
    INFO,
    ERROR,
    WARN,
    FATAL,
    COMPATIBLE_DTYPES,
    BackendException,
    Unspecified,
)

# A singleton that is used to check if an argument was specified.
_unspecified = Unspecified()


class Ros1(eagerx.Backend):

    BACKEND = "ROS1"
    DISTRIBUTED_SUPPORT = True
    MULTIPROCESSING_SUPPORT = True
    COLAB_SUPPORT = True

    @classmethod
    def make(cls, log_level=WARN) -> eagerx.specs.BackendSpec:
        spec = cls.get_specification()
        spec.config.log_level = log_level if isinstance(log_level, str) else eagerx.get_log_level()
        return spec

    def initialize(self):
        _initialize("eagerx_backend_ros1", anonymous=True, log_level=INFO)

        # Set log level
        logger = logging.getLogger("rosout")
        logger.setLevel(self.log_level)

    def Publisher(self, address: str, dtype: str):
        return _Publisher(address, dtype)

    def Subscriber(self, address: str, dtype: str, callback, callback_args=tuple()):
        return _Subscriber(address, dtype, callback, callback_args=callback_args)

    @staticmethod
    def logdebug(msg, *args, **kwargs):
        return rospy.logdebug(msg)

    @staticmethod
    def loginfo(msg, *args, **kwargs):
        return rospy.loginfo(msg)

    @staticmethod
    def logwarn(msg, *args, **kwargs):
        return rospy.logwarn(msg)

    @staticmethod
    def logerr(msg, *args, **kwargs):
        return rospy.logerr(msg)

    def register_environment(self, name: str, force_start: bool, fn: typing.Callable):
        # Check if there already exists an environment
        services = rosservice.get_service_list()
        if f"{name}/environment/shutdown" in services:
            if force_start:
                Ros1.logwarn(f"There already exists an environment named '{name}'. Shutting down existing environment.")
                shutdown_client = rospy.ServiceProxy(f"{name}/environment/shutdown", Trigger)
                shutdown_client.wait_for_service(1)
                shutdown_client(TriggerRequest())
            else:
                msg = (
                    f"There already exists an environment named '{name}'. Exiting now. Set 'force_start=True' if you "
                    "want to shutdown the existing environment."
                )
                Ros1.logerr(msg)
                raise BackendException

        # Setup remote shutdown procedure for environment
        def _remote_shutdown(req: TriggerRequest):
            Ros1.loginfo("Remote shutdown procedure started.")
            fn()
            Ros1.loginfo("Remote shutdown procedure completed.")
            return TriggerResponse(success=True, message="")

        shutdown_srv = rospy.Service(f"{name}/environment/shutdown", Trigger, _remote_shutdown)
        return _ShutdownService(shutdown_srv)

    def delete_param(self, param: str, level: int = 1) -> None:
        try:
            rosparam.delete_param(param)
            Ros1.loginfo(f'Parameters under namespace "{param}" deleted.')
        except rosgraph.masterapi.ROSMasterException as e:
            if level == 0:
                raise BackendException(e)
            elif level == 1:
                Ros1.logwarn(e)
            else:
                pass

    def upload_params(self, ns: str, values: typing.Dict, verbose: bool = False) -> None:
        return rosparam.upload_params(ns, values, verbose=verbose)

    def get_param(self, name: str, default: typing.Any = _unspecified):
        try:
            if isinstance(default, Unspecified):
                return rospy.get_param(name)
            else:
                return rospy.get_param(name, default=default)
        except rosgraph.masterapi.Error as e:
            raise BackendException(e)

    def spin(self):
        return rospy.spin()

    # def on_shutdown(self, fn):
    #     return rospy.core.add_client_shutdown_hook(fn)
    #
    # def signal_shutdown(self, reason):
    #     return rospy.signal_shutdown(reason)


# Private
_LOG_FNS = {
    SILENT: lambda print_str: None,
    DEBUG: rospy.logdebug,
    INFO: rospy.loginfo,
    WARN: rospy.logwarn,
    ERROR: rospy.logerr,
    FATAL: rospy.logfatal,
}

_LOG_LEVELS = {
    SILENT: rospy.DEBUG,
    DEBUG: rospy.DEBUG,
    INFO: rospy.INFO,
    WARN: rospy.WARN,
    ERROR: rospy.ERROR,
    FATAL: rospy.FATAL,
}

_NUMPY_TO_ROS = {key: None for key in COMPATIBLE_DTYPES}
_NUMPY_TO_ROS["str"] = std_msgs.msg.String
_NUMPY_TO_ROS["bool"] = std_msgs.msg.Bool
_NUMPY_TO_ROS["float32"] = std_msgs.msg.Float32MultiArray
_NUMPY_TO_ROS["float64"] = std_msgs.msg.Float64MultiArray
_NUMPY_TO_ROS["int8"] = std_msgs.msg.Int8MultiArray
_NUMPY_TO_ROS["int16"] = std_msgs.msg.Int16MultiArray
_NUMPY_TO_ROS["int32"] = std_msgs.msg.Int32MultiArray
_NUMPY_TO_ROS["int64"] = std_msgs.msg.Int64MultiArray
_NUMPY_TO_ROS["uint8"] = std_msgs.msg.UInt8MultiArray
_NUMPY_TO_ROS["uint16"] = std_msgs.msg.UInt16MultiArray
_NUMPY_TO_ROS["uint32"] = std_msgs.msg.UInt32MultiArray
_NUMPY_TO_ROS["uint64"] = std_msgs.msg.UInt64MultiArray
_ROS_TO_NUMPY = {val: key for key, val in _NUMPY_TO_ROS.items()}

_UNSUPPORTED = [key for key, value in _NUMPY_TO_ROS.items() if value is None]


def _is_roscore_running():
    return rosgraph.is_master_online()


def _launch_roscore():
    if not _is_roscore_running():

        import roslaunch

        uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
        roslaunch.configure_logging(uuid)
        roscore = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)

        try:
            roscore.start()
        except roslaunch.core.RLException:
            Ros1.loginfo(
                "Roscore cannot run as another roscore/master is already running. Continuing without re-initializing the roscore."
            )
            # roscore = None
            raise  # todo: maybe not raise here?
    else:
        Ros1.loginfo(
            "Roscore cannot run as another roscore/master is already running. Continuing without re-initializing the roscore."
        )
        roscore = None
    return roscore


@wraps(rospy.init_node)
def _initialize(*args, log_level=INFO, **kwargs):
    roscore = _launch_roscore()  # First launch roscore (if not already running)
    try:
        rospy.init_node(*args, log_level=_LOG_LEVELS[log_level], **kwargs)
    except rospy.exceptions.ROSException as e:
        Ros1.logwarn(e)
    if roscore:
        atexit.register(roscore.shutdown)
    return roscore


def _dtype_to_ros1_msg_type(dtype: str):
    try:
        msg_type_cls = _NUMPY_TO_ROS[dtype]
        return msg_type_cls
    except KeyError:
        msg = f"Invalid dtype provided that is not supported ({dtype}). Supported dtypes are ({_NUMPY_TO_ROS.keys()})"
        raise KeyError(msg)


# Public Publisher/Subscriber functions


class _Publisher(Publisher):
    def __init__(self, address: str, dtype: str):
        self._address = address
        self._dtype = dtype
        self._msg_type = _dtype_to_ros1_msg_type(dtype)
        self._pub = rospy.Publisher(address, self._msg_type, queue_size=0, latch=True)
        self._name = f"{self._address}"

    def publish(self, msg: typing.Union[float, bool, int, str, np.ndarray, np.number]):
        # Convert python native types to numpy arrays.
        if isinstance(msg, float):
            msg = np.array(msg, dtype="float32")
        elif isinstance(msg, bool):
            msg = np.array(msg, dtype="bool")
        elif isinstance(msg, int):
            msg = np.array(msg, dtype="int64")

        # Check if message complies with space
        if not isinstance(msg, (np.ndarray, np.number, str)):
            Ros1.logerr(f"[publisher][{self._name}]: type(recv)={type(msg)}")
            time.sleep(10000000)

        # Check if dtypes of processed message matches outgoing message_type
        msg_dtype = msg.dtype.name if isinstance(msg, (np.ndarray, np.number)) else type(msg).__name__
        msg_type = _dtype_to_ros1_msg_type(msg_dtype)
        if not msg_type == self._msg_type:
            raise AssertionError(
                f"[publisher][{self._name}]: "
                f"(processed) message dtype ({msg_dtype}) corresponds to msg_type ({msg_type} which is incompatible"
                f" with the registered dtype ({self._dtype}) that corresponds to msg_type ({self._msg_type})."
            )

        # Convert to ROS messages
        shape = msg.shape if isinstance(msg, (np.ndarray, np.number)) else ()
        if msg_dtype == "uint8":
            msg = self._msg_type(data=msg.tobytes("C"))
        elif msg_dtype == "bool":
            msg = self._msg_type(data=msg.item())
        elif msg_dtype == "str":
            msg = self._msg_type(data=msg)
        else:
            msg = self._msg_type(data=msg.reshape(-1))
        for i, d in enumerate(shape):
            dim = std_msgs.msg.MultiArrayDimension(label=str(i), size=d, stride=sum(shape[i:]))
            msg.layout.dim.append(dim)
        # if self._address == "/rx/obj/states/N9/set":
        #     print(f"[publisher][{self._name}]: N9 sent!")
        # elif self._address == "/rx/obj/states/N9/done":
        #     print(f"[publisher][{self._name}]: N9/done sent!")
        return self._pub.publish(msg)

    def unregister(self):
        return self._pub.unregister()


class _Subscriber(Subscriber):
    def __init__(self, address: str, dtype: str, callback, callback_args=tuple()):
        self._address = address
        self._dtype = dtype
        self._msg_type = _dtype_to_ros1_msg_type(dtype)
        self._cb_wrapped = callback
        self._cb_args = callback_args
        self._sub = rospy.Subscriber(address, self._msg_type, callback=self.callback)
        self._name = f"{self._address}"

    def callback(self, msg):
        if not isinstance(msg, tuple(_ROS_TO_NUMPY)):
            Ros1.logerr(f"[subscriber][{self._name}]: type(recv)={type(msg)}")
            time.sleep(10000000)

        # Convert input message to numpy array
        if isinstance(msg, (std_msgs.msg.Bool, std_msgs.msg.String)):
            msg = msg.data
        elif isinstance(msg, std_msgs.msg.UInt8MultiArray):
            s = [d.size for d in msg.layout.dim]
            msg = np.frombuffer(bytes(msg.data), dtype="uint8").reshape(s)
        else:
            s = [d.size for d in msg.layout.dim]
            msg = np.array(msg.data, dtype=_ROS_TO_NUMPY[type(msg)]).reshape(s)

        try:
            # if self._address == "/Pendulum_2_True_0/env/supervisor/end_reset":
            # if self._address == "/rx/obj/states/N9/set":
            #     print(f"[subscriber][{self._name}]: N9 received={msg}!")
            # elif self._address == "/rx/obj/states/N9/done":
            #     print(f"[subscriber][{self._name}]: N9/done received={msg}!")
            self._cb_wrapped(msg, *self._cb_args)
        except rospy.exceptions.ROSException as e:
            self.unregister()
            Ros1.logdebug(f"[subscriber][{self._name}]: Unregistered this subscription because of exception: {e}")

    def unregister(self):
        return self._sub.unregister()


class _ShutdownService(ShutdownService):
    def __init__(self, srv):
        self._srv = srv

    def unregister(self):
        self._srv.shutdown()


assert len(_UNSUPPORTED) == 0, f"Backend `{Ros1.BACKEND}` does not have support for the following dtypes: {_UNSUPPORTED}."

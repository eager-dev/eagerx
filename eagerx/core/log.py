import logging
import inspect
import pickle

# EAGERx
from eagerx.core.constants import (
    DEBUG,
    INFO,
    ERROR,
    WARN,
    FATAL,
)


def get_log_level():
    return logging.getLogger("eagerx").getEffectiveLevel()


def set_log_level(log_level):
    logger = logging.getLogger("eagerx")
    logger.setLevel(log_level)


def _log(msg, level, effective_level=None):
    if level >= get_log_level():
        print(msg)


def logdebug(msg):
    return _log(msg, DEBUG)


def loginfo(msg):
    return _log(msg, INFO)


def logwarn(msg):
    return _log(msg, WARN)


def logerr(msg):
    return _log(msg, ERROR)


def logfatal(msg):
    return _log(msg, FATAL)


def logsilent(msg, *args, **kwargs):
    pass


def frame_to_caller_id(frame):
    caller_id = (
        inspect.getabsfile(frame),
        frame.f_lineno,
        frame.f_lasti,
    )
    return pickle.dumps(caller_id)


class LoggingOnce(object):

    called_caller_ids = set()

    def __call__(self, caller_id):
        if caller_id not in self.called_caller_ids:
            self.called_caller_ids.add(caller_id)
            return True
        return False

# LOG LEVELS
SILENT = 0
DEBUG = 10
INFO = 20
WARN = 30
ERROR = 40
FATAL = 50


NEW_PROCESS = 0
ENVIRONMENT = 1
ENGINE = 2
EXTERNAL = 3


class log:
    SILENT = SILENT
    DEBUG = DEBUG
    INFO = INFO
    WARN = WARN
    ERROR = ERROR
    FATAL = FATAL


# PROCESS
class process:
    #: Spawn the node/engine in a separate process.
    #: Allows parallelization, but increases communication overhead due to the (de)serialization of messages.
    NEW_PROCESS: int = NEW_PROCESS
    #: Spawn the node/engine in the process of the environment.
    ENVIRONMENT: int = ENVIRONMENT
    #: Spawn a node in the process of the engine.
    #: If an :class:`~eagerx.core.entities.EngineNode` requires direct access to the
    #: :attr:`~eagerx.core.entities.Engine.simulator`,
    #: :attr:`~eagerx.core.entities.EngineNode.config`, and
    #: :attr:`~eagerx.core.entities.EngineNode.engine_config`,
    #: it must be spawned in the same process as the engine.
    ENGINE: int = ENGINE
    #: Spawn the node/engine in a separate process. This process is not spawned by the environment.
    #: Instead, the user is responsible for running the executable script with the appropriate arguments.
    #: This allows nodes to run distributed.
    EXTERNAL: int = EXTERNAL


COMPATIBLE_DTYPES = [
    "float32",
    "float64",
    "int8",
    "int16",
    "int32",
    "int64",
    "uint8",
    "uint16",
    "uint32",
    "uint64",
    "bool",
    "str",
]


class BackendException(Exception):
    """
    Base class for exceptions in backend routines
    """

    pass


class Unspecified(object):
    pass

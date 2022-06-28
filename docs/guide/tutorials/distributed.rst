.. _distributed:

***********
Distributed
***********

To launch a node or engine externally on, for example, a different physical machine, you must set its process to
``EXTERNAL``. See :class:`~eagerx.core.constants.process` for more info.
In this case, you as a user are responsible for launching the node/engine.

.. note::
    When using the ``Ros1`` :class:`~eagerx.core.entities.Backend` for running across multiple machines,
    please make sure that the ``ROS_MASTER_URI`` is correctly configured on every machine.
    See `here <http://wiki.ros.org/ROS/Tutorials/MultipleMachines>`_ for more info.


You will have to pass the following arguments

-   Path to the appropriate executable python script (``executable_node.py`` for nodes, ``executable_engine.py`` for engines).

-   ``--backend``: Backend that was selected for the environment (e.g. ``eagerx.backends.ros1/Ros1`` or ``eagerx.backends.single_process/SingleProcess``).

-   ``--loglevel``: The desired log level (as an integer). See :class:`~eagerx.core.constants` for more info.

-   ``--env``: The environment name.

-   ``--name``: The name of the node/engine. For engines, the name is always ``engine``.
    If the node is part of an engine-specific implementation of an object, the node name is ``<object_name>/<node_name>``.

-   ``--object``: If the node is part of an engine-specific implementation of an object, please provide the object name.

For nodes, an example would look like:

.. code::

    python3 <path>/<to>/<package>/eagerx/core/executable_node.py --backend eagerx.backends.ros1/Ros1 --loglevel 20 --env CamEnv --name obj/camera_api --object obj

For an engine, an example would look like:

.. code::

    python3 <path>/<to>/<package>/eagerx/core/executable_engine.py --backend eagerx.backends.ros1/Ros1 --loglevel 20 --env CamEnv --name engine
***************
Troubleshooting
***************

Here we list commonly encountered problems and effective methods for debugging.

-   When developing, users are advised to select the ``SingleProcess`` :class:`~eagerx.core.entities.Backend`.
    Other backends, such as the ``Ros1`` :class:`~eagerx.core.entities.Backend` can make debugging unnecessarily hard due to their distributed capabilities.
    Switch to multi-processing and distributed computing once you have a stable implementation.

-   If you must debug using the ``Ros1`` :class:`~eagerx.core.entities.Backend`, then you are advised to launch all nodes
    in the ENVIRONMENT process. See :class:`~eagerx.core.constants.process` for more info.

-   Live-plotting is currently only supported when the ``Ros1`` :class:`~eagerx.core.entities.Backend` is selected.

-   To run your code using the ``Ros1`` :class:`~eagerx.core.entities.Backend` from within PyCharm,
    make sure to modify your launcher file as described `here <http://wiki.ros.org/IDEs#PyCharm_.28community_edition.29>`_.
    This will also allow you to attach a debugger and set breakpoints.
    Instructions for several other IDEs are also covered in the provided link.

-   Using eagerx with anaconda can produce warnings (see below) when rendering or when using the GUI. This is a known issue that
    is caused by the interaction of pyqtgraph (used in the GUI) and opencv (used for rendering) with Qt libraries. Code seems not
    to break, so as a temporary fix, you are advised to suppress this error. Please file a bug report if eagerx/opencv/gui
    functionality actually breaks.

    .. code::

        QObject::moveToThread: Current thread (0x7fb6c4009eb0) is not the object's thread (0x7fb6c407cf40). Cannot move to
        target thread (0x7fb6c4009eb0).
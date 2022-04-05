********
Training
********

Having created the `OdeBridge <https://github.com/eager-dev/eagerx_ode>`_ :mod:`~eagerx.core.entities.Bridge`, the `Pendulum <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/objects.py>`_ :mod:`~eagerx.core.entities.Object` and the `ButterworthFilter <https://github.com/eager-dev/eagerx/blob/master/eagerx/nodes/butterworth_filter.py>`_ :mod:`~eagerx.core.entities.Node`, we can bring everything together and start to train a policy.
We will do this by first creating a :mod:`~eagerx.core.graph.Graph`, then inspect it using the `GUI <https://github.com/eager-dev/eagerx_gui>`_ and finally train a policy using `Stable-Baselines3 <https://stable-baselines3.readthedocs.io/en/master/>`_.

.. toctree::
   :maxdepth: 2
   :caption: Table of Contents

   ./graph
   ./learn

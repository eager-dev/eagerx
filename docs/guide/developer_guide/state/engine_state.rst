*************
Engine States
*************

In this section we will discuss the concept of an engine state.
We will do so by going through the process of creating an engine state for the *OdeBridge*.
The *OdeBridge* allows to simulate systems based on ordinary differential equations (ODEs).

For the *OdeBridge* we will create two engine states, i.e. the *OdeEngineState* and *OdeParameters* engine states.
These engine states will allow to reset the state of objects and reset the parameters for the ODE integration, respectively.

`Full code is available here. <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_states.py>`_

OdeEngineState
##############

The first engine state will will create is the *OdeEngineState*.
This engine state will be responsible for resetting the states of objects in the *OdeBridge* during a reset of the environment.
Engine states can be created using the :mod:`~eagerx.core.entities.EngineState` base class.
For creating an engine node, we need to implement three abstract methods:

* :func:`~eagerx.core.entities.EngineState.spec`
* :func:`~eagerx.core.entities.EngineState.initialize`
* :func:`~eagerx.core.entities.EngineState.reset`

spec
****

So, first we will implement the spec method.
This method allows to specify default parameters, but also to add custom parameters.
In our case, we do not need to specify parameters, so the implementation is fairly simple:

::

  import numpy as np
  from eagerx.core.entities import EngineState
  import eagerx.core.register as register


  class OdeEngineState(EngineState):
    @staticmethod
    @register.spec("OdeSimState", EngineState)
    def spec(spec):
        spec.initialize(OdeEngineState)

.. note::
  Mind the usage of the :func:`~eagerx.core.register.spec` decorator.
  This decorator is required to register the *OdeEngineState*.
  All entities within EAGERx have to be registered, such that their specification can be created based on their unique id.
  In this decorator we provide a unique id for the engine state (*"OdeSimState"*) and specify the type (:mod:`~eagerx.core.entities.EngineState`).
  Another thing that is worth noting, is that we need to call :func:`~eagerx.core.specs.EngineStateSpec.initialize` with a reference to the class, in this case *OdeEngineState*.
  This will initialize the *spec* object and set default values.


initialize
**********

The :func:`~eagerx.core.entities.EngineState.initialize` method allows to initialize the engine state.
In our case, the only thing we need to do during initialization is to store the object name.

::

  def initialize(self):
    self.obj_name = self.config["name"]

.. note::
  Note that we have access to the :attr:`~eagerx.core.entities.EngineState.config` attribute.
  See :attr:`~eagerx.core.entities.EngineState.config` for more information.

reset
*****

Finally, we will implement the :func:`~eagerx.core.entities.EngineState.reset` method.
This method will be called during a reset and will reset the state of the object.

::

  def reset(self, state, done):
    self.simulator[self.obj_name]["state"] = np.squeeze(state.data)

.. note::
  Note that we have access to the :attr:`~ode_bridge.OdeBridge.simulator` attribute, which is created in the *OdeBridge* class.

Similarly, we can create the *OdeParameters* :mod:`~eagerx.core.entities.EngineState` by implementing the :func:`~eagerx.core.entities.EngineState.spec`, :func:`~eagerx.core.entities.EngineState.initialize` and :func:`~eagerx.core.entities.EngineState.reset` abstract methods.

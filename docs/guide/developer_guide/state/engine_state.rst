*************
Engine States
*************

In this section we will discuss the concept of an engine state.
We will do so by going through the process of creating an engine state for the *OdeBridge*.
The *OdeBridge* allows to simulate systems based on ordinary differential equations (ODEs).

For the *OdeBridge* we will create two engine states, i.e. the *OdeEngineState* and *OdeParameters* engine states.
These engine states will allow to reset the state of objects and reset the parameters for the ODE integration, respectively.


OdeEngineState
##############

The first engine state will will create is the *OdeEngineState*.
This engine state will be responsible for resetting the states of objects in the *OdeBridge* during a reset of the environment.
Creating engine states can be done by inheriting from :mod:`eagerx.core.entities.EngineState`.
For creating an engine node, we need to implement three abstract methods:

* :meth:`eagerx.core.entities.EngineState.spec`
* :meth:`eagerx.core.entities.EngineState.initialize`
* :meth:`eagerx.core.entities.EngineState.reset`

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

.. note::  Mind the usage of the :meth:`eagerx.core.register.spec` decorator.
  This decorator is required to register the *OdeEngineState*.
  All entities within EAGERx have to be registered, such that they can be used based on their unique id.
  In this decorator we provide a unique id for the engine state (*"OdeSimState"*) and specify the type (:mod:`eagerx.core.entities.EngineState`).
  Another thing that is worth noting, is that we need to call :meth:`eagerx.core.register.spec.initialize` with a reference to the class, in this case *OdeEngineState*.
  This will initialize the *spec* object and set default values.

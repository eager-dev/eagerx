.. _ode_engine:

*********
OdeEngine
*********

We will start by creating a file called `engine.py <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine.py>`_.
Here we will define the *OdeEngine*, which will be a subclass of the :class:`~eagerx.core.entities.Engine` class.
This class has six abstract methods:

* :func:`~eagerx.core.entities.Engine.spec`, here we will specify the *OdeEngine*'s parameters in a configuration :class:`~eagerx.core.specs.EngineSpec` object.
* :func:`~eagerx.core.entities.Engine.initialize`, here we determine how the *OdeEngine* initializes using the specification that is created in the :func:`~eagerx.core.entities.Engine.spec` function..
* :func:`~eagerx.core.entities.Engine.add_object`, here we will specify how objects are added.
* :func:`~eagerx.core.entities.Engine.pre_reset`, here we prepare a reset of the *OdeEngine*.
* :func:`~eagerx.core.entities.Engine.reset`, here we perform the reset routine before the start of an episode.
* :func:`~eagerx.core.entities.Engine.callback`, here we define what will happen every time step.
  In our case we will integrate the ODEs of each object.

`Full code is available here. <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine.py>`_

.. figure:: /_static/img/engine.svg
  :align: center
  :alt: alternate text
  :figclass: align-center

  In this section we will discuss the concept of a :class:`~eagerx.core.entities.Engine`.
  The :class:`~eagerx.core.entities.Engine` connects the :class:`~eagerx.core.env.EagerxEnv`, :class:`~eagerx.core.graph_engine.EngineGraph` and :class:`~eagerx.core.entities.EngineState` to the physics engine/real world.

spec
####

First we will define the :func:`~eagerx.core.entities.Engine.spec` method.
In this method we will "specify" a number of parameters of the *OdeEngine*.

We can make a distinction between standard parameters and custom parameters.
First of all, there are the standard parameters for the :class:`~eagerx.core.entities.Engine` class:

* :attr:`~eagerx.core.entities.Engine.rate`
* :attr:`~eagerx.core.entities.Engine.process`
* :attr:`~eagerx.core.entities.Engine.sync`
* :attr:`~eagerx.core.entities.Engine.real_time_factor`
* :attr:`~eagerx.core.entities.Engine.simulate_delays`
* :attr:`~eagerx.core.entities.Engine.log_level`

Secondly, we will define some parameters that are custom for the OdeEngine.
We will use these to set some of the parameters of the `odeint <https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.odeint.html>`_ method from :class:`scipy.integrate` which we will use to integrate the ODEs.
These custom parameters are:

* **rtol**: *float*, The input parameters rtol and atol determine the error control performed by the solver.
* **atol**: *float*, The input parameters rtol and atol determine the error control performed by the solver.
* **hmax**: *float*, The maximum absolute step size allowed.
* **hmin**: *float*, The minimum absolute step size allowed.
* **mxstep**: *int*, Maximum number of (internally defined) steps allowed for each integration point in t.

We can define the default values for all of these parameters using the spec function as follows:

::

  # OTHER
  from typing import Optional, Dict, Union, List
  from scipy.integrate import odeint

  # ROS IMPORTS
  import rospy
  from std_msgs.msg import UInt64
  from genpy.message import Message

  # RX IMPORTS
  from eagerx.core.constants import process, ERROR
  import eagerx.core.register as register
  from eagerx.core.entities import Engine
  from eagerx.core.specs import EngineSpec
  from eagerx.utils.utils import Msg, get_attribute_from_module

  class OdeEngine(Engine):
    @staticmethod
    @register.spec("OdeEngine", Engine)
    def spec(
          spec: EngineSpec,
          rate,
          process: Optional[int] = process.NEW_PROCESS,
          sync: Optional[bool] = True,
          real_time_factor: Optional[float] = 0,
          simulate_delays: Optional[bool] = True,
          log_level: Optional[int] = ERROR,
          rtol: float = 2e-8,
          atol: float = 2e-8,
          hmax: float = 0.0,
          hmin: float = 0.0,
          mxstep: int = 0,
      ):
          # Modify default engine params
          spec.config.rate = rate
          spec.config.process = process
          spec.config.sync = sync
          spec.config.real_time_factor = real_time_factor
          spec.config.simulate_delays = simulate_delays
          spec.config.log_level = log_level
          spec.config.color = "magenta"

          # Add custom params
          custom = dict(rtol=rtol, atol=atol, hmax=hmax, hmin=hmin, mxstep=mxstep)
          spec.config.update(custom)

.. note::
  There are couple of things that are worth mentioning here.
  First of all, we see the *staticmethod* and :func:`~eagerx.core.register.spec` decorators.
  You are probably familiar with the first one, but the second might need some explanation.
  We use the :func:`~eagerx.core.register.spec` decorator to create an identifier for this engine, i.e. "OdeEngine".
  Also, it will allow us to directly modify default engine parameters that are stored in the *spec* object of type :class:`~eagerx.core.specs.EngineSpec`.
  Custom arguments correspond to the arguments of the :func:`~eagerx.core.entities.Engine.initialize` method as we will see later on.

  Also worth noting, is that we can see that there are two ways to set parameters, i.e. by setting them directly or by using the :func:`~eagerx.core.view.update` method.

initialize
##########

Next, we will define the :func:`~eagerx.core.entities.Engine.initialize` method.
This method is called with the custom parameters we have just specified (rtol, atol, hmax, hmin, mxstep).
This function will be executed before the first time the :func:`~eagerx.core.entities.Engine.callback`, :func:`~eagerx.core.entities.Engine.add_object`, :func:`~eagerx.core.entities.Engine.reset` and :func:`~eagerx.core.entities.Engine.pre_reset` methods are run.
So all attributes that are defined here, are accessible in those methods.
The logic in this routine depends on the physics engine/simulator you would like to interface.
In this case, the simulator is particularly simple, i.e. we will only integrate ODEs.
Therefore, all we need to do to initialize the *OdeEngine* is to define two dictionaries:

::

  def initialize(self, rtol, atol, hmax, hmin, mxstep):
        # Initialize any simulator here, that is passed as reference to each engine node
        self.odeint_args = dict(rtol=rtol, atol=atol, hmax=hmax, hmin=hmin, mxstep=mxstep)
        self.simulator = dict()

.. note::
  Note that the parameters under "custom params" correspond to the signature of the :func:`~eagerx.core.entities.Engine.initialize` method.
  In this way, we can easily use these parameters to initialize the *OdeEngine* node.
  We will use the *simulator* attribute to keep track of the objects and their ODEs, states and inputs.
  This *simulator* object is a special object, since it will be shared among all the engine nodes of type :class:`~eagerx.core.entities.EnigneNode`.
  In this way, we create a reference simulator attribute in the :class:`~eagerx.core.entities.Engine`.

add_object
##########

The :func:`~eagerx.core.entities.Engine.add_object` method initializes each object in the engine.
In our case, this means that we will add a dictionary to the *simulator* attribute with the object's name as key.
This dictionary contains information about the object that we will need for integration of the ODE.
First of all, we need a reference to the function that describes the ODE of the object (*ode*).
Secondly, we allow users to provide a reference to a function that defines the Jacobian (*Dfun*), in order to speed up integration.
This *Dfun* will be optional, such that we can also simulate ODEs without a provided Jacobian.
Also, we allow users to specify parameters that can be used to set arguments of the *ode*:

::

  @register.engine_config(ode=None, ode_params=list())
  def add_object(self, config, engine_config, node_params, state_params):

      # Extract relevant agnostic params
      obj_name = config["name"]
      ode = get_attribute_from_module(engine_config["ode"])
      Dfun = get_attribute_from_module(engine_config["Dfun"]) if "Dfun" in config else None

      # Create new object, and add to simulator
      self.simulator[obj_name] = dict(
          ode=ode,
          Dfun=Dfun,
          state=None,
          input=None,
          ode_params=engine_config["ode_params"],
      )

.. note::
  Here the :func:`~eagerx.utils.utils.get_attribute_from_module` function is just a helper function to import an attribute from a module based on a string that is defined as "[module_name]/[attribute]".
  Again, note the :func:`~eagerx.core.register.engine_config` decorator in which the *ode* and *ode_params* parameters are registered.
  Every :class:`~eagerx.core.entities.Object` interfaced with this :class:`~eagerx.core.entities.Engine` will have to specify these parameters.
  The engine receives these parameters via the ``engine_config`` argument.
  The ``engine_config`` object is meant to be used for all parameters that are engine specific.
  The agnostic params should be defined in the ``config`` object.

pre_reset
#########

The :func:`~eagerx.core.entities.Engine.pre_reset` method allows to define procedures that will be run before starting a reset.
This could for example be useful when some routine should be performed in order to be able to reset, e.g. switching controllers or pausing/starting a simulator.
In our case, we do not need to do this, so this will be a simple pass:

::

  def pre_reset(self, **kwargs: Optional[Msg]):
        pass

reset
#####

The :func:`~eagerx.core.entities.Engine.reset` method is called by the user before the start of an episode.
This allows to reset the state of the *OdeEngine*.
In our case, we are not adding a state to the *OdeEngine*.
However, this could be done, for example to vary the integration parameters over episodes as a form of domain randomization.
In our case, we will not do this.
Therefore, the reset method will also be a simple pass:

::

  @register.states()
  def reset(self, **kwargs: Optional[Msg]):
      pass

.. note::
  Note the :func:`~eagerx.core.register.states` decorator.
  If we wanted the *OdeEngine* to have a state, we could add it using this decorator.

callback
########

Finally, we will specify how we integrate the ODEs every time step.
This will be done in the :func:`~eagerx.core.entities.Engine.callback` method.
As mentioned before, we will use :func:`scipy.integrate.odeint` for this.
The callback will be executed at the specified :attr:`~eagerx.core.entities.Engine.rate`.

::

  @register.outputs(tick=UInt64)
  def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
      for _obj_name, sim in self.simulator.items():
          # Get the input, set by engine nodes as we will see later on.
          input = sim["input"]
          ode = sim["ode"]
          Dfun = sim["Dfun"]
          x = sim["state"]

          # Get the ode_params that are set by engine states as we will see later on.
          ode_params = sim["ode_params"]

          # If no input was set, return without stepping the simulator.
          if input is None
            return

          # Integrate the ODE
          sim["state"] = odeint(
              ode,
              x,
              [0, 1.0 / self.rate],
              args=(input, *ode_params),
              Dfun=Dfun,
              **self.odeint_args,
          )[-1]

.. note::
  Using the :func:`~eagerx.core.register.outputs` decorator, we specify all the outputs of the *OdeEngine* node.
  In our case, the output is a simple "tick", see :func:`~eagerx.core.entities.Engine.callback` for more information.

Next, we will create the engine nodes.

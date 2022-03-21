OdeBridge
#########

We will start by creating a file called *bridge.py*.
Here we will define the *OdeBridge*, which will be a subclass of the :mod:`~eagerx.core.entities.Bridge` class.
This class has six abstract methods:

* :func:`~eagerx.core.entities.Bridge.spec`, here we will specify the OdeBridge's parameters.
* :func:`~eagerx.core.entities.Bridge.initialize`, here we will specify how the OdeBridge is initialized.
* :func:`~eagerx.core.entities.Bridge.add_object`, here we will specify how objects are added.
* :func:`~eagerx.core.entities.Bridge.pre_reset`, here we prepare a reset of the OdeBridge.
* :func:`~eagerx.core.entities.Bridge.reset`, here we perform the reset.
* :func:`~eagerx.core.entities.Bridge.callback`, here we define what will happen every time step.
  In our case we will integrate the ODEs of each object.


spec
****

First we will define the *spec* method.
In this method we will "specify" a number of parameters of the *OdeBridge*.

We can make a distinction between standard parameters and custom parameters.
First of all, there are the standard parameters for the :mod:`~eagerx.core.entities.Bridge` class:

* :attr:`~eagerx.core.entities.Bridge.rate`
* :attr:`~eagerx.core.entities.Bridge.process`
* :attr:`~eagerx.core.entities.Bridge.is_reactive`
* :attr:`~eagerx.core.entities.Bridge.real_time_factor`
* :attr:`~eagerx.core.entities.Bridge.simulate_delays`
* :attr:`~eagerx.core.entities.Bridge.log_level`

Secondly, we will define some parameters that are custom for the OdeBridge.
We will use these to set some of the parameters of the `odeint <https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.odeint.html>`_ method from :mod:`scipy.integrate` which we will use to integrate the ODEs.
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
  from eagerx.core.entities import Bridge
  from eagerx.core.specs import BridgeSpec
  from eagerx.utils.utils import Msg, get_attribute_from_module

  class OdeBridge(Bridge):
    @staticmethod
    @register.spec("OdeBridge", Bridge)
    def spec(
          spec: BridgeSpec,
          rate,
          process: Optional[int] = process.NEW_PROCESS,
          is_reactive: Optional[bool] = True,
          real_time_factor: Optional[float] = 0,
          simulate_delays: Optional[bool] = True,
          log_level: Optional[int] = ERROR,
          rtol: float = 2e-8,
          atol: float = 2e-8,
          hmax: float = 0.0,
          hmin: float = 0.0,
          mxstep: int = 0,
      ):
          # Performs all the steps to fill-in the params with registered info about all functions.
          spec.initialize(OdeBridge)

          # Modify default bridge params
          spec.config.rate = rate
          spec.config.process = process
          spec.config.is_reactive = is_reactive
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
  We use the :func:`~eagerx.core.register.spec` decorator to create an identifier for this bridge, i.e. "OdeBridge".
  Also, it will allow us to directly modify default bridge parameters that are stored in the *spec* object of type :mod:`~eagerx.core.specs.BridgeSpec`.
  Note that we first need to run :func:`~eagerx.core.specs.BridgeSpec.initialize`.

  Also worth noting, is that the custom parameters are set differently, i.e. using the :func:`~eagerx.core.view.update` method.
  This is necessary, because only the standard parameters from :mod:`~eagerx.core.entities.Bridge` can be set directly.
  Any custom parameter has to be added using the :func:`~eagerx.core.view.update` method.

initialize
**********

Next, we will define the :func:`~eagerx.core.entities.Bridge.initialize` method.
This method is called with the custom parameters we have just specified (rtol, atol, hmax, hmin, mxstep).
This function will be executed before the first time the :func:`~eagerx.core.entities.Bridge.callback`, :func:`~eagerx.core.entities.Bridge.add_object`, :func:`~eagerx.core.entities.Bridge.reset` and :func:`~eagerx.core.entities.Bridge.pre_reset` methods are run.
So all attributes that are defined here, are accessible in those methods.
All we need to do to initialize the *OdeBridge* is to define two dictionaries:

::

  def initialize(self, rtol, atol, hmax, hmin, mxstep):
        # Initialize any simulator here, that is passed as reference to each engine node
        self.odeint_args = dict(rtol=rtol, atol=atol, hmax=hmax, hmin=hmin, mxstep=mxstep)
        self.simulator = dict()

.. note::
  Note that the custom parameters we have added to the *spec* of type :mod:`~eagerx.core.specs.BridgeSpec` using the :func:`~eagerx.core.view.update` method are now arguments to the :func:`~eagerx.core.entities.Bridge.initialize` method.
  In this way, we can easily use these parameters to initialize the *OdeBridge* node.
  We will use the *simulator* attribute to keep track of the objects and their ODEs, states and inputs.

add_object
**********

The :func:`~eagerx.core.entities.Bridge.add_object` method initializes each object in the bridge.
In our case, this means that we will add a dictionary to the *simulator* attribute with the object's name as key.
This dictionary contains information about the object that we will need for integration of the ODE.
First of all, we need a reference to the function that describes the ODE of the object (*ode*).
Secondly, we allow users to provide a reference to a function that defines the Jacobian (*Dfun*), in order to speed up integration.
This *Dfun* will be optional, such that we can also simulate ODEs without a provided Jacobian.
Also, we allow users to specify parameters that can be used to set arguments of the *ode*:

::

  @register.bridge_config(ode=None, ode_params=list())
  def add_object(self, config, bridge_config, node_params, state_params):

      # Extract relevant agnostic_params
      obj_name = config["name"]
      ode = get_attribute_from_module(bridge_config["ode"])
      Dfun = get_attribute_from_module(bridge_config["Dfun"]) if "Dfun" in config else None

      # Create new object, and add to simulator
      self.simulator[obj_name] = dict(
          ode=ode,
          Dfun=Dfun,
          state=None,
          input=None,
          ode_params=bridge_config["ode_params"],
      )

.. note::
  Here the :func:`~eagerx.utils.utils.get_attribute_from_module` function is just a helper function to import an attribute from a module based on a string that is defined as "[module_name]/[attribute]".
  Again, note the :func:`~eagerx.core.register.bridge_config` decorator in which the *ode* and *ode_params* parameters are added to the *bridge_config* object.
  The *bridge_config* object is meant to be used for all parameters that are bridge specific.
  The agnostic params should be defined in the *config* object.

pre_reset
*********

The :func:`~eagerx.core.entities.Bridge.pre_reset` method allows to define procedures that will be run before starting a reset.
In our case, we do not need to do this, so this will be a simple pass:

::

  def pre_reset(self, **kwargs: Optional[Msg]):
        pass

reset
*****

The :func:`~eagerx.core.entities.Bridge.reset` method is called every time a reset is performed.
This allows to reset the state of the *OdeBridge*.
In our case, we are not adding a state to the *OdeBridge*.
However, this could be done, for example to vary the integration parameters over episodes as a form of domain randomization.
In our case, we will not do this.
Therefore, the reset method will also be a simple pass:

::

  @register.states()
  def reset(self, **kwargs: Optional[Msg]):
      pass

.. note::
  Note the :func:`~eagerx.core.register.states` decorator.
  If we wanted the *OdeBridge* to have a state, we could add it using this decorator.

callback
********

Finally, we will specify how we integrate the ODEs every time step.
This will be done in the :func:`~eagerx.core.entities.Bridge.callback` method.
As mentioned before, we will use :func:`scipy.integrate.odeint` for this.
The callback will be executed at the specified :attr:`~eagerx.core.entities.Bridge.rate`.

::

  @register.outputs(tick=UInt64)
  def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
      for _obj_name, sim in self.simulator.items():
          input = sim["input"]
          ode = sim["ode"]
          Dfun = sim["Dfun"]
          x = sim["state"]
          ode_params = sim["ode_params"]
          if x is not None and input is not None:
              sim["state"] = odeint(
                  ode,
                  x,
                  [0, 1.0 / self.rate],
                  args=(input, *ode_params),
                  Dfun=Dfun,
                  **self.odeint_args,
              )[-1]

.. note::
  Using the :func:`~eagerx.core.register.outputs` decorator, we specify all the outputs of the *OdeBridge* node.
  In our case, the output is a simple "tick", see :func:`~eagerx.core.entities.Bridge.callback` for more information.


Next, we will create the engine nodes:

.. toctree::
   :maxdepth: 1

   ./engine_node

************
Engine Nodes
************

In this section, we will show how to create an :class:`~eagerx.core.entities.EngineNode`.
Engine nodes are nodes that interact with the :class:`~eagerx.core.entities.Bridge` node and define the behaviour of sensors and actuators.
An :class:`~eagerx.core.entities.EngineNode` is often bridge-specific, since here is defined how actions are applied and observations are obtained.
We will clarify the concept of engine nodes in this section by going through the process of creating the engine nodes for the *OdeBridge*.
This :class:`~eagerx.core.entities.Bridge` allows to simulate systems based on ordinary differential equations (ODEs).
In the engine nodes for the *OdeBridge*, we will define how inputs and outputs are send to and from the *OdeBridge*.
We will define three classes: *OdeOutput*, *OdeInput* and *ActionApplied*.
Each of these classes will be a subclass of the *EngineNode* class.
Here we will go into detail on how to the *OdeInput* :class:`~eagerx.core.entities.EngineNode` is created.

`Full code is available here. <https://github.com/eager-dev/eagerx_ode/blob/master/eagerx_ode/engine_nodes.py>`_

.. figure:: /_static/img/engine_node.svg
  :align: center
  :alt: alternate text
  :figclass: align-center

  In this section we will discuss the concept of a :class:`~eagerx.core.entities.EngineNode`.
  In the engine nodes, we create an implementation of actuators and sensors for a specific :class:`~eagerx.core.entities.Bridge`.
  The :class:`~eagerx.core.entities.EngineNode` can be added to an :class:`~eagerx.core.graph_engine.EngineGraph`.

OdeInput
########

First we will define an :class:`~eagerx.core.entities.EngineNode` for setting the input for the *OdeBridge*.
We can do this by using the :class:`~eagerx.core.entities.EngineNode` base class.
For the :class:`~eagerx.core.entities.EngineNode` base class, there are four abstract methods:

* :func:`~eagerx.core.entities.EngineNode.spec`, here we will specify the parameters of *OdeInput*.
* :func:`~eagerx.core.entities.EngineNode.initialize`, here we will specify how the *OdeInput* node is initialized.
* :func:`~eagerx.core.entities.EngineNode.reset`, here we will specify how to reset the state of the *OdeInput* node.
* :func:`~eagerx.core.entities.EngineNode.callback`, here we will define what this node will do every clock tick.
  In this case, it will use the last input/action in the *OdeBridge* node.

spec
****

The :func:`~eagerx.core.entities.EngineNode.spec` method can be used to specify default parameters for engine nodes and to assign an id to the node.
Since we need a reference to the simulator (the *OdeBridge*), we will also specify here that we run this node in the bridge process per default.
If this node is run in another process, we won't have a reference to the *simulator* attribute from the *OdeBridge* and will not be able to pass inputs easily to the *OdeBridge* node.
We also specify that this node has two input and one output, which respectively are "tick", "action" and "action_applied".
The "tick" input is required, since it ensures that the *OdeInput* :class:`~eagerx.core.entities.EngineNode` is synchronized with the *OdeBridge* :class:`~eagerx.core.entities.Bridge`.
Also, we add a custom parameter called *default_action*, which will allow to specify a default action that will be applied in case it is not overwritten.
The spec method now looks as follows:

::

  from typing import Optional, List
  import numpy as np

  # IMPORT ROS
  from std_msgs.msg import UInt64, Float32MultiArray

  # IMPORT EAGERX
  from eagerx.core.constants import process
  from eagerx.utils.utils import Msg
  from eagerx.core.entities import EngineNode
  import eagerx.core.register as register


  class OdeInput(EngineNode):
    @staticmethod
    @register.spec("OdeInput", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        default_action: List,
        color: Optional[str] = "green",
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(OdeInput)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate  # Rate at which the callback is called
        spec.config.process = process  # This should always be the process of the Bridge
        spec.config.inputs = ["tick", "action"]  # Set default inputs
        spec.config.outputs = ["action_applied"]  # Set default outputs

        # Set custom node params
        spec.config.default_action = default_action

.. note::
  Note the use of the :func:`~eagerx.core.register.spec` decorator to register the id of this :class:`~eagerx.core.entities.EngineNode`.
  This basically allows to use this node in objects using the id.

initialize
**********

Next, we will implement the :func:`~eagerx.core.entities.EngineNode.initialize` method.
In this method we will set the object name, the default action and check whether the node is launched in the correct process:

::

  def initialize(self, default_action):
    assert (
        self.process == process.BRIDGE
    ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
    self.obj_name = self.config["name"]
    self.default_action = np.array(default_action)

.. note::
  Note that the parameter *default_action*, which we added to the *spec* object of type :class:`~eagerx.core.specs.NodeSpec` becomes an argument to the :func:`~eagerx.core.entities.EngineNode.initialize` method.

reset
*****

We will use the :func:`~eagerx.core.entities.EngineNode.reset` method to reset the object's input to the default input:

::

  @register.states()
    def reset(self):
        self.simulator[self.obj_name]["input"] = np.squeeze(np.array(self.default_action))

.. note::
  Since we do not want the *OdeInput* to have any states to reset, the :func:`~eagerx.core.register.states` decorator is used without any arguments.

callback
********

At the specified :attr:`~eagerx.core.entities.EngineNode.rate` of the *OdeInput* node, the :func:`~eagerx.core.entities.EngineNode.callback` function will be called.
In this callback we want to update the action that will be applied by the *OdeBridge* based on the latest action we have received.
Here, we will also define the inputs and outputs of the *OdeInput* node and their message types.
This is necessary in order to set up communication pipelines in the background.
In our case, the inputs are the bridge tick "tick" with message type :class:`~std_msgs.msg.UInt64` and the action "action" which will be a :class:`~std_msgs.msg.Float32MultiArray`.
In code, this is implemented as follows:

::

    @register.inputs(tick=UInt64, action=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action: Optional[Float32MultiArray] = None,
    ):
        # Set action in simulator for next step.
        self.simulator[self.obj_name]["input"] = np.squeeze(action.msgs[-1].data)

        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])

.. note::
  Note that the message type as provided using the :func:`~eagerx.core.register.inputs` and :func:`~eagerx.core.register.outputs` decorators, should be ROS message types.
  For more information, see the documentation on :func:`~eagerx.core.entities.EngineNode.callback`.
  Also, the "tick" input ensures that this :func:`~eagerx.core.entities.EngineNode.callback` is synchronized with the :class:`~eagerx.core.entities.Bridge`.

Similarly, we can create the engine nodes *OdeOutput* and *ActionApplied* for obtaining the output from the *OdeBridge* simulator and obtaining the value for the action that is applied.
The *ActionApplied* will allow other nodes to listen to the action that is applied in the simulator.
This can be useful for example when some form of preprocessing is applied on the action before it is applied to the environment.
Then, this node can be used to feedback the applied action as an observation to the environment.

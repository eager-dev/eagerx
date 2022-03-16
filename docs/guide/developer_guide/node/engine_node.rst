************
Engine Nodes
************

In this section, we will show how to create engine nodes.
Engine nodes are nodes that interact with bridge nodes and can be used to for example define the behaviour of sensors and actuators.
We will clarify the concept of engine nodes in this section by going through the process of creating the engine nodes for the *OdeBridge*.
This bridge allows to simulate systems based on ordinary differential equations (ODEs).
In the engine nodes for the *OdeBridge*, we will define how inputs and outputs are send to and from the *OdeBridge*.
We will define three classes: *OdeOutput*, *OdeInput* and *ActionApplied*.
Each of these classes will be a subclass of the *EngineNode* class.
Here we will go into detail on how to the *OdeInput* engine node is created.


OdeInput
########

First we will define an engine_node for sending inputs to the *OdeBridge*.
For the *EngineNode*, there are four abstract classes:

* *spec*, here we will specify the OdeInput's parameters.
* *initialize*, here we will specify how the OdeInput node is initialized.
* *reset*, here we will specify how to reset the OdeInput node.
* *callback*, here we will define what this node will do every clock tick.
  In this case, it will send the last input/action to the *OdeBridge* node.

spec
****

The spec method can be used to specify default parameters for engine nodes and to assign an id to the node.
Since we need a reference to the simulator (the *OdeBridge*), we will also specify here that we run this node in the bridge process per default.
If this node is run in another process, we won't have a reference to the *OdeBridge* and will not be able to pass inputs easily to the *OdeBridge* node.
We also specify that this node has two input and one output, which respectively are "tick", "action" and "action_applied".
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
        process: Optional[int] = process.BRIDGE,
        color: Optional[str] = "green",
    ):
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(OdeInput)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "action"]
        spec.config.outputs = ["action_applied"]

        # Set custom node params
        spec.config.default_action = default_action

Note the use of the *register.spec* decorator to register the id of this *EngineNode*.
This basically allows to use this node in objects based by referencing to this id.

initialize
**********

Next, we will implement the *initialize* method.
In this method we will set the object name, the default action and check whether the node is launched in the correct process:

::

  def initialize(self, default_action):
    assert (
        self.process == process.BRIDGE
    ), "Simulation node requires a reference to the simulator, hence it must be launched in the Bridge process"
    self.obj_name = self.agnostic_params["name"]
    self.default_action = np.array(default_action)

reset
*****

We will use the reset method to reset the object's input to the default input:

::

  @register.states()
    def reset(self):
        self.simulator[self.obj_name]["input"] = np.squeeze(np.array(self.default_action))

Since we do not have any states that we want to reset, the *register.states* decorator is used without any arguments.

callback
********

Every tick of this node, the callback function will be called.
In this callback we want to update the action that will be applied by the *OdeBridge* based on the latest action we have received.
Here, we will also define the inputs and outputs of the *OdeInput* node and their message types.
This is necessary in order to create publishers and subscribers in the background for sending and receiving these messages.
In our case, the inputs are the bridge tick "tick" with message type *UInt64* and the action "action" which will be a *Float32MultiArray*.
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

Similarly, we can create the engine nodes *OdeOutput* and *ActionApplied* for obtaining the output from the *OdeBridge* simulator and obtaining the value for the action that is applied.
The *ActionApplied* will allow other nodes to listen to the action that is applied in the simulator.
This can be useful for example when some form of preprocessing is applied on the actions and the action that is applied is could be an observation of the environment in that case. 

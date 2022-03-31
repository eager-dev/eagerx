****
Node
****

In this section, we will discuss the concept of a :mod:`~eagerx.core.entities.Node`.
A node can be used to process data at a desired rate.
This could for example be a classifier to detect objects in an image or a PID controller that reduces a control error.
Here, we will go through the process of creating such a :mod:`~eagerx.core.entities.Node`.
We will create the *ButterworthFilter* :mod:`~eagerx.core.entities.Node`, which can be used to filter signals.

The :mod:`~eagerx.core.entities.Node` base class has four abstract methods we need to implement:

- :mod:`~eagerx.core.entities.Node.spec`
- :mod:`~eagerx.core.entities.Node.initialize`
- :mod:`~eagerx.core.entities.Node.reset`
- :mod:`~eagerx.core.entities.Node.callback`

`Full code is available here. <https://github.com/eager-dev/eagerx/blob/master/eagerx/nodes/butterworth_filter.py>`_

.. autoclass:: eagerx.nodes.butterworth_filter.ButterworthFilter
    :members:


spec
####

::

  from typing import Optional
  from scipy.signal import butter, sosfilt

  # IMPORT ROS
  from std_msgs.msg import Float32MultiArray

  # IMPORT EAGERX
  import eagerx.core.register as register
  from eagerx.utils.utils import Msg
  from eagerx.core.entities import Node, Processor, SpaceConverter
  from eagerx.core.constants import process


  class ButterworthFilter(Node):
      @staticmethod
      @register.spec("ButterworthFilter", Node)
      def spec(
          spec,
          name: str,
          rate: float,
          index: int = 0,
          N: int = 2,
          Wn: float = 1,
          btype: str = "lowpass",
          process: Optional[int] = process.NEW_PROCESS,
          color: Optional[str] = "grey",
      ):
          # Performs all the steps to fill-in the params with registered info about all functions.
          spec.initialize(ButterworthFilter)

          # Modify default node params
          spec.config.name = name
          spec.config.rate = rate
          spec.config.process = process
          spec.config.color = color
          spec.config.inputs = ["signal"]
          spec.config.outputs = ["filtered"]

          # Modify custom node params
          spec.config.N = N
          spec.config.Wn = Wn
          spec.config.btype = btype

          # Add converter & space_converter
          spec.inputs.signal.window = "$(config N)"
          spec.inputs.signal.converter = Processor.make("GetIndex_Float32MultiArray", index=index)
          spec.inputs.signal.space_converter = SpaceConverter.make("Space_Float32MultiArray", [-3], [3], dtype="float32")

initialize
##########

::

  def initialize(self, N, Wn, btype):
      for i in self.inputs:
          if i["name"] == "signal":
              assert (
                  int(i["window"]) >= N
              ), "The window size of the signal {} is too small to create a filter with order {}.".format(i["window"], N)
      self.filter = butter(N, Wn, btype, output="sos", fs=self.rate)
      self.N = N

reset
#####

::

  @register.states()
  def reset(self):
    pass


callback
########

::

  @register.inputs(signal=Float32MultiArray)
  @register.outputs(filtered=Float32MultiArray)
  def callback(self, t_n: float, signal: Optional[Msg] = None):
    msgs = signal.msgs
    if len(msgs) >= self.N:
        unfiltered = [msgs[i].data[0] for i in range(-self.N, 0)]
        filtered = msgs[-1].data if None in unfiltered else [sosfilt(self.filter, unfiltered)[-1]]
    elif len(msgs) > 0:
        filtered = msgs[-1].data
    else:
        filtered = [0.0]
    return dict(filtered=Float32MultiArray(data=filtered))

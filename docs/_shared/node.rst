****
Node
****

In this section, we will discuss the concept of a :class:`~eagerx.core.entities.Node`.
A node can be used to process data at a desired rate.
This could for example be a classifier to detect objects in an image or a PID controller that reduces a control error.
Here, we will go through the process of creating such a :class:`~eagerx.core.entities.Node`.
We will create the *ButterworthFilter* :class:`~eagerx.core.entities.Node`, which can be used to filter signals.

The :class:`~eagerx.core.entities.Node` base class has four abstract methods we need to implement:

- :class:`~eagerx.core.entities.Node.spec`
- :class:`~eagerx.core.entities.Node.initialize`
- :class:`~eagerx.core.entities.Node.reset`
- :class:`~eagerx.core.entities.Node.callback`

`Full code is available here. <https://github.com/eager-dev/eagerx/blob/master/eagerx/nodes/butterworth_filter.py>`_

.. figure:: /_static/img/node.svg
  :align: center
  :alt: alternate text
  :figclass: align-center

  In this section we will discuss the concept of a :class:`~eagerx.core.entities.Node`.
  It can be added to a :class:`~eagerx.core.graph.Graph` and is engine-agnostic.

spec
####

Here we define the specification of the *ButterworthFilter*.
Since we will make use of the `Butterworth filter implementation from scipy <https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html>`_, we want to initialize the node with the arguments of this implementation.
Because the signature of the :func:`~eagerx.core.entities.Node.initialize` is defined within the :func:`~eagerx.core.entities.Node.spec` method, we add the parameters *N*, *Wn* and *btype* to the :attr:`~eagerx.core.specs.NodeSpec.config`.
These are the order of the filter, the critical frequency of the filter and the filter type, respectively.
Furthermore, we add a converter to the :class:`~eagerx.core.specs.NodeSpec` of the *ButterworthFilter*, since we want to apply this filter on a scalar signal, while the input to the filter might be multidimensional.
Therefore, we make use of the :class:`~eagerx.converters.ros_processor.GetIndex_Float32MultiArray` :class:`~eagerx.core.entities.Processor`, which selects the entry of a `Float32MultiArray <http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html>`_.
Finally, we will set a :class:`~eagerx.core.entities.SpaceConverter`, such that we can directly :func:`~eagerx.core.graph.Graph.connect` the *ButterworthFilter* to an action without having to define the `OpenAI Gym Space <https://gym.openai.com/docs/#spaces>`_ every time.

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
          # Modify default node params
          spec.config.name = name
          spec.config.rate = rate
          spec.config.process = process
          spec.config.color = color
          spec.config.inputs = ["signal"]
          spec.config.outputs = ["filtered"]

          # Modify custom node params
          spec.config.N = N  # The order of the filter.
          spec.config.Wn = Wn  # The critical frequency or frequencies.
          spec.config.btype = btype  # {‘lowpass’, ‘highpass’, ‘bandpass’, ‘bandstop’} The type of filter. Default is ‘lowpass’.

          # Add converter & space_converter
          spec.inputs.signal.window = "$(config N)"
          spec.inputs.signal.converter = Processor.make("GetIndex_Float32MultiArray", index=index)
          spec.inputs.signal.space_converter = SpaceConverter.make("Space_Float32MultiArray", [-3], [3], dtype="float32")

.. note::
  Mind the usage of the :func:`~eagerx.core.register.spec` decorator.
  This specifies the ID of the :class:`~eagerx.core.entities.Node`.
  Also, mind the way the *window* is set.
  Here we specify that the window size is equal to the parameter *N*, which is the order of the filter.
  The syntax *$(config [parameter_name])* allows to use a parameter as variable for setting another parameter.


initialize
##########

Within the :func:`~eagerx.core.entities.Node.initialize` method, we will initialize the filter.

::

  def initialize(self, N, Wn, btype):
      for i in self.inputs:
          if i["name"] == "signal":
              assert (
                  int(i["window"]) >= N
              ), "The window size of the signal {} is too small to create a filter with order {}.".format(i["window"], N)
      self.filter = butter(N, Wn, btype, output="sos", fs=self.rate)
      self.N = N

.. note::
  Mind that the signature of the :func:`~eagerx.core.entities.Node.initialize` method is specified by adding parameters to :attr:`~eagerx.core.specs.NodeSpec.config` wihtin :func:`~eagerx.core.entities.Node.spec`.

reset
#####

The :func:`~eagerx.core.entities.Node.reset` method is called by the user at the beginning of an episode.
Here the state of the :class:`~eagerx.core.entities.Node` can be reset.
However, in our case this is not needed.

::

  @register.states()
  def reset(self):
    pass

.. note::
  Mind the usage of the :func:`~eagerx.core.register.states` decorator.
  If the :class:`~eagerx.core.entities.Node` would have had a state that should be reset, it should be registered here.
  We leave it empty because there is no state to reset.

callback
########

The :func:`~eagerx.core.entities.Node.callback` method is called with at the :attr:`~eagerx.core.entities.Node.rate` of the :class:`~eagerx.core.entities.Node`.
This is were the actual signal processing takes place.

::

  @register.inputs(signal=Float32MultiArray)
  @register.outputs(filtered=Float32MultiArray)
  def callback(self, t_n: float, signal: Optional[Msg] = None):
    msgs = signal.msgs

    # Only apply filtering if we have received enough messages (more than the order of the filter)
    if len(msgs) >= self.N:
        unfiltered = [msgs[i].data[0] for i in range(-self.N, 0)]
        filtered = msgs[-1].data if None in unfiltered else [sosfilt(self.filter, unfiltered)[-1]]
    # If we haven't received enough messages, no filtering is applied
    elif len(msgs) > 0:
        filtered = msgs[-1].data
    # If no messages were received, return 0.0
    else:
        filtered = [0.0]
    return dict(filtered=Float32MultiArray(data=filtered))

.. note::
  Mind the usage of the :func:`~eagerx.core.register.inputs` and :func:`~eagerx.core.register.outputs` decorators.
  These register the inputs :attr:`~eagerx.core.entities.Node.inputs` and :attr:`~eagerx.core.entities.Node.outputs` of the :class:`~eagerx.core.entities.Node` and their message types.
  Also, note that the :func:`~eagerx.core.entities.Node.callback` method has the ``t_n`` argument, which is the time passed (seconds) since last reset.

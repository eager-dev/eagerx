.. _space_converter_guide:
***************
Space Converter
***************

In this section we will discuss the concept of a :class:`~eagerx.core.entities.SpaceConverter`.
A space converter can be used to create `Openai Gym Spaces <https://gym.openai.com/docs/#spaces>`_ for messages and define how we can convert them from and to a *numpy.ndarray*, which is the default data type in OpenAI Gym.
In this section we will go through the process of creating the *Space_AngleDecomposition*, which will allow to convert a *Float32MultiArray* to a *numpy.ndarray*.
At the same time, we will decompose one of the entries of the *Float32MultiArray* into a sine and cosine component.
This space converter can be used when dealing with angular positions, since learning on the sine and cosine is often more efficient due to the discontinuities in the angular position.

The :class:`~eagerx.core.entities.SpaceConverter` base class has two class variables:

- :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_A`
- :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_B`

and has 5 abstract methods:

- :func:`~eagerx.core.entities.SpaceConverter.spec`
- :func:`~eagerx.core.entities.SpaceConverter.initialize`
- :func:`~eagerx.core.entities.SpaceConverter.get_space`
- :func:`~eagerx.core.entities.SpaceConverter.A_to_B`
- :func:`~eagerx.core.entities.SpaceConverter.B_to_A`

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/converters.py>`_

MSG_TYPE_A and MSG_TYPE_B
#########################

The class variables :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_A` and :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_B` specify the two message types that will be converted from one into the other.
For the Gym space, we need an *numpy.ndarray*, so MSG_TYPE_A will be of this type.
The second message type will be a `Float32MultiArray <http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html>`_, since this ROS message can be used for multidimensional data communication over ROS topics.


::

  # ROS IMPORTS
  from std_msgs.msg import Float32MultiArray

  # RX IMPORTS
  import eagerx.core.register as register
  from eagerx import Processor, SpaceConverter
  from eagerx.core.specs import ConverterSpec
  import numpy as np
  from gym.spaces import Box


  class Space_AngleDecomposition(SpaceConverter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

spec
####

The :func:`~eagerx.core.entities.SpaceConverter.spec` method can be used to specify with which arguments the :class:`~eagerx.core.entities.SpaceConverter` will be initialized.
In our case, we add *low*, *high* and *dtype* to the :attr:`~eagerx.core.specs.ConverterSpec.config`.

::

  @staticmethod
  @register.spec("Space_AngleDecomposition", SpaceConverter)
  def spec(spec: ConverterSpec, low=None, high=None, dtype="float32"):
      # Initialize spec with default arguments
      spec.initialize(Space_AngleDecomposition)
      params = dict(low=low, high=high, dtype=dtype)
      spec.config.update(params)

.. note::

  Mind the use of the :func:`~eagerx.core.register.spec` decorator.

initialize
##########

Next, we implement the :func:`~eagerx.core.entities.SpaceConverter.initialize` method.
Here, the arguments are the ones we have just defined in the :func:`~eagerx.core.entities.SpaceConverter.spec` method: *low*, *high* and *dtype*.

::

  def initialize(self, low=None, high=None, dtype="float32"):
      self.low = np.array(low, dtype=dtype)
      self.high = np.array(high, dtype=dtype)
      self.dtype = dtype

get_space
#########

The :func:`~eagerx.core.entities.SpaceConverter.get_space` method should be used to define the Gym space.

::

  def get_space(self):
      return Box(self.low, self.high, dtype=self.dtype)

A_to_B
######

The :func:`~eagerx.core.entities.SpaceConverter.A_to_B` method takes as an argument a message of type :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_A` and converts it into :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_B`.

::

  def A_to_B(self, msg):
    return Float32MultiArray(data=msg)

B_to_A
######

The :func:`~eagerx.core.entities.SpaceConverter.B_to_A` method takes as an argument a message of type :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_B` and converts it into :attr:`~eagerx.core.entities.SpaceConverter.MSG_TYPE_A`.
In our case, we also decompose the angle here, which will be the first entry of the array.

::

  def B_to_A(self, msg):
      angle = msg.data[0]
      return np.concatenate(([np.sin(angle), np.cos(angle)], msg.data[1:]), axis=0)

make
####

In order to use this :mod:`~eagerx.core.entities.SpaceConverter`, the user should call the :func:`~eagerx.core.entities.SpaceConverter.make` method with the arguments of the :func:`~eagerx.core.entities.SpaceConverter.spec` method.

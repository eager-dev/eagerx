.. _converter:

*********
Converter
*********

In this section we will discuss the concept of a :class:`~eagerx.core.entities.Converter`.
A converter can be used to convert them convert messages from one type to another.
In this section we will go through the process of creating the *Ndarray_Float32MultiArray* :class:`~eagerx.core.entities.Converter`, which will allow to convert a *Float32MultiArray* to a *numpy.ndarray*.

The :class:`~eagerx.core.entities.Converter` base class has two class variables:

- :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A`
- :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`

and has 4 abstract methods:

- :func:`~eagerx.core.entities.Converter.spec`
- :func:`~eagerx.core.entities.Converter.initialize`
- :func:`~eagerx.core.entities.Converter.A_to_B`
- :func:`~eagerx.core.entities.Converter.B_to_A`

MSG_TYPE_A and MSG_TYPE_B
#########################

The class variables :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A` and :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B` specify the two message types that will be converted from one into the other.
In this example, MSG_TYPE_A will be of type *numpy.ndarray*.
The second message type will be a `Float32MultiArray <http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html>`_, since this ROS message can be used for multidimensional data communication over ROS topics.


::

  # ROS IMPORTS
  from std_msgs.msg import Float32MultiArray

  # RX IMPORTS
  import eagerx.core.register as register
  from eagerx import Converter
  import numpy as np


  class Ndarray_Float32MultiArray(Converter):
    MSG_TYPE_A = np.ndarray
    MSG_TYPE_B = Float32MultiArray

spec
####

The :func:`~eagerx.core.entities.SpaceConverter.spec` method can be used to specify with which arguments the :class:`~eagerx.core.entities.Converter` will be initialized.
In our case, we add *dtype* to the :attr:`~eagerx.core.specs.ConverterSpec.config`.

::

  @staticmethod
  @register.spec("Ndarray_Float32MultiArray", Converter)
  def spec(spec: ConverterSpec, dtype="float32"):
      spec.config.dtype = dtype

.. note::

  Mind the use of the :func:`~eagerx.core.register.spec` decorator.

initialize
##########

Next, we implement the :func:`~eagerx.core.entities.Converter.initialize` method.
Here, the arguments is the ones we have just defined in the :func:`~eagerx.core.entities.Converter.spec` method: *dtype*.

::

  def initialize(self, dtype="float32"):
      self.dtype = dtype

A_to_B
######

The :func:`~eagerx.core.entities.Converter.A_to_B` method takes as an argument a message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A` and converts it into :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B`.

::

  def A_to_B(self, msg):
    return Float32MultiArray(data=msg)

B_to_A
######

The :func:`~eagerx.core.entities.Converter.B_to_A` method takes as an argument a message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE_B` and converts it into :attr:`~eagerx.core.entities.Converter.MSG_TYPE_A`.
In our case, we convert the `Float32MultiArray` into a

::

  def B_to_A(self, msg):
      return np.array(msg.data, dtype=self.dtype)

make
####

In order to use this :mod:`~eagerx.core.entities.Converter`, the user should call the :func:`~eagerx.core.entities.Converter.make` method with the arguments of the :func:`~eagerx.core.entities.Converter.spec` method.

.. _processor:

*********
Processor
*********

In this section we will discuss the concept of a :class:`~eagerx.core.entities.Processor`.
A processor can be used to convert messages without changing the data type.
Therefore, creating a :class:`~eagerx.core.entities.Processor` is similar to creating a :class:`~eagerx.core.entitites.Converter`, but instead of having two message types, there is only one.
In this section we will go through the process of creating the *AngleDecomposition* :class:`~eagerx.core.entities.Processor`.
This will decompose one of the entries of a *Float32MultiArray* into a sine and cosine component.
This processor can be used when dealing with angular positions, since learning on the sine and cosine is often more efficient due to the discontinuities in the angular position.

The :class:`~eagerx.core.entities.Processor` base class has one class variable:

- :attr:`~eagerx.core.entities.Processor.MSG_TYPE`

and has 3 abstract methods:

- :func:`~eagerx.core.entities.Processor.spec`
- :func:`~eagerx.core.entities.Processor.initialize`
- :func:`~eagerx.core.entities.Processor.convert`

MSG_TYPE
########

The class variable :attr:`~eagerx.core.entities.Converter.MSG_TYPE` is the type of the message that will processed.
The message type will be a `Float32MultiArray <http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html>`_, since this ROS message can be used for multidimensional data communication over ROS topics.


::

  # ROS IMPORTS
  from std_msgs.msg import Float32MultiArray

  # RX IMPORTS
  import eagerx.core.register as register
  from eagerx import Processor
  import numpy as np


  class AngleDecomposition(Processor):
    MSG_TYPE = Float32MultiArray
spec
####

The :func:`~eagerx.core.entities.Processor.spec` method can be used to specify with which arguments the :class:`~eagerx.core.entities.Processor` will be initialized.
In our case, we add *angle_idx* to the :attr:`~eagerx.core.specs.ConverterSpec.config`.

::

  @staticmethod
  @register.spec("AngleDecomposition", Processor)
  def spec(spec, angle_idx: int = 0):
      spec.config.angle_idx = angle_idx

.. note::

  Mind the use of the :func:`~eagerx.core.register.spec` decorator.

initialize
##########

Next, we implement the :func:`~eagerx.core.entities.Processor.initialize` method.
Here, the arguments is the ones we have just defined in the :func:`~eagerx.core.entities.Processor.spec` method: *angle_idx*.

::

  def initialize(self, dtype="float32"):
      self.angle_idx = angle_idx

convert
#######

The :func:`~eagerx.core.entities.Processor.convert` method takes as an argument a message of type :attr:`~eagerx.core.entities.Converter.MSG_TYPE` and processes it.

::

  def convert(self, msg):
      if msg.data == []:
          return msg
      angle = msg.data[self.angle_idx]
      new_data = np.concatenate(([np.sin(angle), np.cos(angle)], msg.data[self.angle_idx + 1 :]), axis=0)
      return Float32MultiArray(data=new_data)

make
####

In order to use this :mod:`~eagerx.core.entities.Processor`, the user should call the :func:`~eagerx.core.entities.Processor.make` method with the arguments of the :func:`~eagerx.core.entities.Processor.spec` method.

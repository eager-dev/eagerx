Implementing the real Pendulum
##############################


Now that we have an implementation for the pendulum in simulation, we are ready to create an implementation for the real system.
For this we will use the `*RealEngine* <https://github.com/eager-dev/eagerx_reality>`_.
This engine can be used as an interface to real-world systems.
In order to be able to use the real pendulum system in EAGERx, we need to create an :mod:`~eagerx.core.entities.EngineNode` for each actuator and sensor we have.
Also, we need to create an :mod:`~eagerx.core.entities.EngineState` for each state we want to reset.
In our case, we will create a *PendulumInput* actuator and a *PendulumOutput* sensor, `which are defined here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/real/engine_nodes.py>`_.
The *PendulumInput* :mod:`~eagerx.core.entities.EngineNode` is responsible for sending commands to the pendulum system, while *PendulumOutput* is responsible for reading the sensor data.
These two engine nodes will communicate with a `ROS interface we have created for the system <https://github.com/eager-dev/dcsc_setups>`_.
Also, we create an :mod:`~eagerx.core.entities.EngineState` for resetting the system.
This "reset" will send a random action to the system and sleep afterwards.
Therefore, we call this :mod:`~eagerx.core.entities.EngineState` *RandomActionAndSleep*, `for which the code can be found here <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/eagerx_dcsc_setups/pendulum/real/engine_states.py>`_.

Next, we will create a :mod:`~eagerx.core.entities.SpaceConverter`, which will allow us to use the decomposed angle as an observation, instead of the wrapped angle.
This will most likely result in faster training times, because the wrapped angle has discontinuities which frustrate learning.

Furthermore, we will create a :mod:`~eagerx.core.entities.Node` that will allow to preprocess the actions that we send to the pendulum system.
That is to say, we don't want to send high frequency voltage signals to the DC motor of the pendulum system, because it might damage the system.
Therefore, we will create a low-pass Butterworth filter node, that will prevent sending high frequency voltage signals.

*****
Learn
*****

Next, we will train and hope to see the pendulum swing up in the end!
First we will train in simulation and afterwards we will fine tune the policy on the real system, since we will probably have some model inaccuracies.

`Full code is available here. <https://github.com/eager-dev/eagerx_dcsc_setups/blob/master/examples/example_real.py>`_

Initialize the Bridges
######################

After creating the :mod:`~eagerx.core.graph.Graph`, we will also :func:`~eagerx.core.entities.Bridge.make` the *OdeBridge* and *RealBridge*.

::

  # Define bridges
  bridge_ode = Bridge.make('OdeBridge', rate=rate, is_reactive=True, real_time_factor=0, process=process.NEW_PROCESS)
  bridge_real = Bridge.make('RealBridge', rate=rate, is_reactive=True, process=process.NEW_PROCESS)

EagerxEnv
#########

Next, we will create a :attr:`~eagerx.core.env.EagerxEnv.step_fn` function.
Here we will calculate the reward and check for termination conditions.
We terminate the episode if the number of steps is larger than 500.

::

  # Define step function
  def step_fn(prev_obs, obs, action, steps):
    state = obs["observation"][0]
    # Calculate reward
    sin_th, cos_th, thdot = state
    th = np.arctan2(sin_th, cos_th)
    cost = th ** 2 + 0.1 * (thdot / (1 + 10 * abs(th))) ** 2
    # Determine done flag
    done = steps > 500
    # Set info:
    info = dict()
    return obs, -cost, done, info

.. note::
  We can obtain the angular position and angular velocity from the *obs* dictionary.
  Remember that we used the :func:`~eagerx.core.graph.Graph.connect` method with value for the argument *observation* set to "observation".
  Therefore, we this data is stored under the key "observation".


We will then initialize two times a :mod:`~eagerx.core.env.EagerxEnv`: one with the *OdeBridge* and one with the *RealBridge*.

::

  # Initialize Environment
  real_env = Flatten(EagerxEnv(name='real', rate=rate, graph=graph, bridge=bridge_real, step_fn=step_fn))
  simulation_env = Flatten(EagerxEnv(name='ode', rate=rate, graph=graph, bridge=bridge_ode, step_fn=step_fn))

Train in Simulation
###################

Now we will train in simulation!
We will do this for 450 seconds and save the resulting model.


::

  # Initialize learner (kudos to @araffin)
  model = sb.SAC("MlpPolicy", simulation_env, verbose=1)

  # First train in simulation
  simulation_env.render('human')
  model.learn(total_timesteps=int(450*rate))
  simulation_env.close()

  # Evaluate for 30 seconds in simulation
  rospy.loginfo('Start simulation evaluation!')
  obs = simulation_env.reset()
  for i in range(int(30 * rate)):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = simulation_env.step(action)
    if done:
        obs = simulation_env.reset()

  model.save('simulation')
  simulation_env.shutdown()

Fine Tuning in Reality
######################

We can load the saved model and fine tune it on the real system in order to successfully swing up the real pendulum.

::

  # Train on real system
  model = sb.SAC.load('simulation', env=real_env, ent_coef="auto_0.1")
  real_env.render('human')

  # Evaluate on real system
  rospy.loginfo('Start zero-shot evaluation!')
  obs = real_env.reset()
  for i in range(int(90 * rate)):
     action, _states = model.predict(obs, deterministic=True)
     obs, reward, done, info = real_env.step(action)
     real_env.render()
     if done:
         obs = real_env.reset()

  # Fine-tune policy
  rospy.loginfo('Start fine-tuning!')
  model.learn(total_timesteps=int(1020*rate))
  model.save('real')

  # Evaluate on real system
  rospy.loginfo('Start fine-tuned evaluation!')
  obs = real_env.reset()
  while True:
     action, _states = model.predict(obs, deterministic=True)
     obs, reward, done, info = real_env.step(action)
     real_env.render()
     if done:
         obs = real_env.reset()

And that is it!
We have trained a policy in simulation and fine tuned it on the real system.

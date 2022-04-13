Introduction
############

In this tutorial we will go through the process of learning a pendulum to swing up using EAGERx.
We will touch upon the following subjects in this tutorial:

* How to create a bridge
* How to create an object
* How to create a space converter
* How to use the GUI
* How to create a Graph
* How to learn a policy using your environment

Objectives
**********

Our aim is to create an implementation of a pendulum system we have in our lab and perform experiments with it in EAGERx.
This pendulum system is often used in our department for performing experiment, e.g. for evaluating novel algorithms.
It is non-trivial to design a controller that swings up the pendulum from an arbitrary state and to stabilize it in the upright position, due to nonlinear dynamics and underactuation.
Because of this, it is also an interesting problem!

We would like to achieve the following objectives:

* We want to be able to simulate experience in simulation, since real-world interactions are time consuming and result in wear and tear.
* We want to be able to fuse experience from simulation and from the real system.
* We want to have an implementation that is modular, such that it can be used easily for other experiments.

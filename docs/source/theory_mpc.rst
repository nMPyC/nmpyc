Basics of model predictive control
===================================

**Model predictive control (MPC)** is an optimized-based method for obtaining an approximately optimal feedback control for an optimal control problem on an infinite or finite time horizon. The basic idea of MPC is to predict the future behavior of the controlled system over a finite time horizon and compute an optimal control input that, while ensuring satisfaction of given system constraints, minimizes the objective function. In each sampling instant a finite horizon open-loop optimal control problem is solved to calculate the control input. More precisley, this control input is used to define the feedback which is applied to the system until the next sampling instant, at whicht the horizon is shifted and the procedure is repeated again.

Optimal control problems
--------------------------------
In order to describe the functionality of MPC we consider optimal control problems. To this end, we consider possibly nonlinear difference equations of the form 

.. math::

       x(k+1,x_0) &= f(x(k,x_0),u(k)), \quad k = 0,\dots,N-1, \\
       x(0) &= x_0

with :math:`N\in\mathbb{N}` or discretized differential equations.

Further, we impose nonempty state and input constraint sets :math:`\mathbb{X}\subseteq\mathbb{R}^{n}` and :math:`\mathbb{U}\subseteq\mathbb{R}^m`, respectively, as well as a nonempty terminal constraint set :math:`\mathbb{X}_0\subseteq\mathbb{R}^n`.

Now we use optimal control to determine :math:`u(0),\dots,u(N-1)`. For this reason, we fix a stage cost :math:`\ell:\mathbb{X}\times\mathbb{U}\to\mathbb{R}` which may be a very general function and a optional terminal cost :math:`F:\mathbb{X}\to\mathbb{R}`. Regardless which cost function is used the objective function is defined by

.. math::
       J^N(x_0,u(\cdot)):=\sum_{k=0}^{N-1}\ell(x(k,x_0),u(k))

without terminal cost or by   

.. math::
       J^N(x_0,u(\cdot)):=\sum_{k=0}^{N-1}\ell(x(k,x_0),u(k))+ F(x(N,x_0))

with terminal cost.

In summary, an optimal control problem without terminal conditions is given by 

.. math::
  :nowrap:

       \begin{equation}
       \begin{split}
              \min_{u(\cdot)\in\mathbb{U}}J^N(x_0,u(\cdot)) &= \sum_{k=0}^{N-1}\ell(x(k,x_0),u(k))\\
              \text{s.t.}\quad x(k+1,x_0)&=f(x(k,x_0),u(k)),\quad k = 0,\dots, N-1\\
              x(0)&= x_0\\
              x&\in\mathbb{X}
       \end{split}
       \end{equation}

and an optimal control problem with terminal conditions is given by

.. math::
  :nowrap:

       \begin{equation}
       \begin{split}
              \min_{u(\cdot)\in\mathbb{U}}J^N(x_0,u(\cdot)) &= \sum_{k=0}^{N-1}\ell(x(k,x_0),u(k))+F(x(N,x_0))\\
              \text{s.t.}\quad x(k+1,x_0)&=f(x(k,x_0),u(k)),\quad k = 0,\dots, N-1\\
              x(0)&= x_0\\
              x\in\mathbb{X},\quad & x(N,x_0)\in\mathbb{X}_0
       \end{split}
       \end{equation}

Additionally, with **nMPyC** it is possible to add constraints to the optimal control problem.

The basic MPC algorithm
------------------------
Regardless of the type of the optimal control problem, the MPC algorithm is given by:

At each time instant :math:`j=0,1,2,\dots:`
       1. Measure the state :math:`x(j)\in\mathbb{X}` of the system.
       2. Set :math:`x_0:=x(j)`, solve the optimal control problem (with or without terminal conditions) and denote the obtained optimal control sequence by :math:`u^\star(\cdot)\in\mathbb{U}^N(x_0)`.
       3. Define the MPC-feedback value :math:`\mu^N(x(j)):=u^\star(0)\in\mathbb{U}` and use this control value in the next sampling period (apply the feedback to the system).

Notes and extensions
---------------------
A special case of an optimal control problem is a linear-quadratic problem. There, the stage cost is defined as a quadratic function and the dynamics are linear.

.. note::
       **nMPyC** supports a time dependent formulation of optimal control problem. Hence, all functions, as :math:`f, \ell, F`, can depend on the time instance :math:`j`.

.. note::
       **nMPyC** supports also discounted optimal control problems. In the discrete case the objective is defined as 
       .. math::
              J^N(x_0,u(\cdot)):=\sum_{k=0}^{N-1}\beta^k\ell(x(k,x_0),u(k))
       with :math:`\beta\in(0,1)` the discount factor.

Further reading
----------------
For further reading and more theoretical insights we kindly refer to


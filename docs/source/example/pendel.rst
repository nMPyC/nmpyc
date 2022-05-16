Inverted Pendulum
==================

We consider the mechanical model of an inverted rigid pendulum mounted on a carriage.

By means of physical laws an "exact" differential equation model can be derived.
However, since in our case we want to obtain a linear quadratic problem, 
we linearize the differential equation at the origin.
Thus we obtain the system dynamics defined by 

.. math::

   \dot{x}(t) = \left(\begin{array}{cccc} 
      0 & 1 & 0 & 0 \\
      g & -k & 0 & 0 \\
      0 & 0 & 0 & 1 \\
      0 & 0 & 0 & 0
   \end{array}\right) x(t) + \left(\begin{array}{c} 
      0  \\
      1  \\
      0  \\
      1 
   \end{array}\right) u(t).

Here, the state vector :math:`x \in \mathbb{R}^4` consists of 4 components. :math:`x_1` corresponds to the angle :math:`\psi` of the pendulum, which which increases counterclockwise, where :math:`x_1 = 0` corresponds to the upright pendulum. :math:`x_2` is the angular velocity, :math:`x_3` the position of the carriage and :math:`x_4` its velocity. 
The control :math:`u` is the acceleration of the carriage. 
The constant :math:`k=0.1` describes the friction of the pendulum and the constant :math:`g \approx 9.81 m/s^2` is the acceleration due to gravity.

Since the system dynamics are linear, we can initialize them using the LQP method. 

.. code-block:: python
   
   g = 9.81
   k = 0.1
   A = mpc.array([[0, 1, 0, 0], 
               [g, -k, 0, 0], 
               [0, 0, 0, 1],
               [0, 0, 0, 0]])
   B = mpc.array([0, 1, 0, 1])
   system = mpc.system.LQP(A, B, 4, 1, 'continuous', 
                           sampling_rate=0.1, method='rk4')

Note that we have to use one of the fixed step methods euler, heun or rk4 as integration method if we want to exploit the linear structure of the problem later in the optimization.




Haunschmied
============

To examplify a discounted problem we consider a 2d variant of an investment problem.

Our system dynamics for this problem are given by 

.. math::

   \dot{x}_1(t) &= x_2(t) - \sigma x_1(t) \\ 
   \dot{x}_2(t) &= u(t)

where we set :math:`\sigma = 0.25` for our calculations.

To implement the dynamics we now have to initialize a function that implemnts the right hand side of the dynamics.

.. code-block:: python

   sigma = 0.25
   def f(x,u):
       y = nmpyc.array(2)
       y[0] = x[1]-sigma *x[0]
       y[1] = u
       return y

After that, the nMPyC system object can be set by calling

.. code-block:: python

   system = nmpyc.system(f, 2, 1, 'continuous', sampling_rate=h, method='heun')

To model the payoff of the investment problem we assume th stagecost

.. math::

   \ell(x,u) = R(x_1) - c(x_2) - v(u)

where :math:`R(x_1) = k_1 \sqrt{x_1} - x_1/(1+k_2 x_1^4)` is a evenue function of the firm with a
convex segment due to increasing returns. :math:`c(x_2) = c_1 x_2 + c_2 x_2^2/2` denotes adjustment costs
of investment and :math:`v(u) = \alpha u^2/2` represents adjustment costs of the change of investment.
The convex segment in the payoff function just mentioned is likely to generate two domains
of attraction.
Additionally we choose :math:`k_1=2`, :math:`k_2=0.0117`, :math:`c_1=0.75`, :math:`c_2=2.5` and :math:`\alpha=12` for our computations.

With the nMPyC package the implemnetiation of the objective corresponding to this costs can be done as follws.

.. code-block:: python

   def l(x,u):
       R = k1*x[0]**(1/2)-x[0]/(1+k2*x[0]**4)
       c = c1*x[1]+(c2*x[1]**2)/2
       v = (alpha*u[0]**2)/2
       return -(R - c - v)

   objective = nmpyc.objective(l)

Since this problem is unconstrained we can now initialize our model by 

.. code-block:: python

   model = nmpyc.model(objective,system)


Chemical Reactor
=================

We consider a single first-order, irreversible chemical reaction in an isothermal CSTR

.. math::

   A \to B \quad r=kc_A
   
in which :math:`k` is the rate constant. The material balances and the system data are provided in :cite:p:`Diehl2011` and is given by the nonlinear model

.. math::
   :nowrap:
   
   \begin{equation*}
      \begin{split}
         c^{A}_{t+1}&=c_t^{A}+h\left(\frac{Q_t}{V}(c_f^{A}-c_t^{A})-k_r{c_t^{A}}\right)\\
         c^{B}_{t+1}&=c_t^{B}+h\left(\frac{Q_t}{V}(c_f^{B}-c_t^{B})+k_r{c_t^{B}}\right),
      \end{split}
   \end{equation*}
   
in which :math:`c_A\geq 0` and :math:`c_B\geq 0` are the molar concentrations of :math:`A` and :math:`B` respectively, and :math:`Q\leq 20` (L/min) is 
the flow through the reactor. The constants and their meanings are given in table below.

================================ =============================  =====================================  ================
   Reactor constants
-----------------------------------------------------------------------------------------------------------------------
            \                             \                                 value                           unit
================================ =============================  =====================================  ================
feed concentration of :math:`A`   :math:`c_f^{A}`                                  1                        mol/L
feed concentration of :math:`B`   :math:`c_f^{B}`                                  0                        mol/L
volume of the reactor             :math:`V_R`                                     10                          L
rate constant                     :math:`k_r`                                     1.2                    L/(mol min)
equilibrium                       :math:`(c_e^{A},c_e^B,Q_e)`    :math:`(\frac 1 2, \frac 1 2, 12)`
start value                       :math:`(c_0^{A},c_0^B)`        :math:`(0.4, 0.2)`
================================ =============================  =====================================  ================

To initialize the system dynamics in our code, we must first define a function that implements :math:`f(x,u)`.

.. code-block:: python

   # parameters
   V = 10.
   cf_A = 1.
   cf_B = 0.
   k_r = 1.2

   def f(x,u):
       y = mpc.array(2)
       h = 0.5
       y[0] = x[0] + h*((u[0]/V) *(cf_A - x[0]) - k_r*x[0])
       y[1] = x[1] + h*((u[0]/V) *(cf_B - x[1]) + k_r*x[1])
       return y

Further, we consider the stage cost given by 

.. math::
   :nowrap:
   
   \begin{align*}
      \ell (c_t^{A},c_t^{B},Q_t)&=\frac 1 2\vert c_t^{A}-\frac 1 2\vert^2+\frac 1 2 \vert c_t^B-\frac 1 2\vert^2+\frac 1 2 \vert Q_t -12 \vert^2\\
   \end{align*}

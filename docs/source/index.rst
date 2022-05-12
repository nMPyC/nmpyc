Welcome to nMPyC's documentation!
===================================

**nMPyC** is a Python library for solving optimal control problems via model predictive control (MPC).

**nMPyC** can be understood as a blackbox method. The user can only enter the desired optimal control problem without having much knowledge of the theory of model predictive control or its implementation in Python. Nevertheless, for an advanced user, there is the possibility to adjust all parameters.

This library supports a variety of discretization methods and optimizers including `CasADi <https://web.casadi.org/>`_ and `SciPy <https://scipy.org/>`_ solvers.

In summary, **nMPyC**
   - solves nonlinear finite horizon optimal control problems 
   - solves nonlinear optimal control problems with model predicitve control (MPC)
   - uses algorithmic differentation via `CasADi <https://web.casadi.org/>`_
   - can chose between different discretization methods
   - can chose between different solvers for nonlinear optimization (depending on the problem)
   - supports time-varying optimal control problems
   - supports the special structure of linear-quadratic optimal control problems
   - supports discounted optimal control problems
   - can save and load the simulation results



.. note::

   This project is under active development.


Contents
--------

.. toctree::
   :maxdepth: 2

   install
   start
   theory_mpc
   api
   examples
   templates
   faq
   citation
   references

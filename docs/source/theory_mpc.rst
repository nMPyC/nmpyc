Basics of model predictive control
===================================

*Model predictive control (MPC)* is an optimized-based method for obtaining an approximately optimal feedback control for an optimal control problem on an infinite or finite time horizon. The basic idea of MPC is to predict the future behavior of the controlled system over a finite time horizon and compute an optimal control input that, while ensuring satisfaction of given system constraints, minimizes the objective function. In each sampling instant a finite horizon open-loop optimal control problem is solved to calculate the control input. More precis, this control input is used to define the feedback which is applied to the system until the next sampling instant, at whicht the horizon is shifted and the procedure is repeated again.

Optimal control problems
--------------------------------
In order to describe the functionality of MPC we consider optimal control problems. To this end, we consider difference equations of the form 


To be more precise, the control input is calculated by solving at each sampling instant a finite horizon open-loop optimal control problem; the first part of the resulting optimal input trajectory is then applied to the system until the next sampling instant, at which the horizon is shifted and the whole procedure is repeated again.
**nMPyC** is a Python library for solving optimal control problems via model predictive control (MPC).

**nMPyC** can be understood as a blackbox method. The user can only enter the desired optimal control problem without having much knowledge of the theory of model predictive control or its implementation in Python. Nevertheless, for an advanced user, there is the possibility to adjust all parameters.

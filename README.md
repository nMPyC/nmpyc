# nMPyC

**nMPyC** is a Python library for solving optimal control problems via model predictive control (MPC).

**nMPyC** can be understood as a blackbox method. The user can only enter the desired optimal control problem without having much knowledge of the theory of model predictive control or its implementation in Python. Nevertheless, for an advanced user, there is the possibility to adjust all parameters.

This library supports a variety of discretization methods and optimizers including [CasADi](https://web.casadi.org/>) and [SciPy](https://scipy.org/>) solvers.

In summary, **nMPyC**
   - solves nonlinear finite horizon optimal control problems 
   - solves nonlinear optimal control problems with model predicitve control (MPC)
   - uses algorithmic differentation via [CasADi](https://web.casadi.org/)
   - can chose between different discretization methods
   - can chose between different solvers for nonlinear optimization (depending on the problem)
   - supports time-varying optimal control problems
   - supports the special structure of linear-quadratic optimal control problems
   - supports discounted optimal control problems
   - can save and load the simulation results

The **nMPyC** software is Python based and works therefore on any OS with a Python distribution. 
**nMPyC** has been developed by Jonas Schießl and Lisa Krügel under the supervision of Prof. Lars Grüne at the [Chair of Applied Mathematic](https://num.math.uni-bayreuth.de/en/index.html) of University of Bayreuth. 
**nMPyC** is a further devolpement in Python of the [Matlab code](http://numerik.mathematik.uni-bayreuth.de/~lgruene/nmpc-book/matlab_nmpc.html) that was implemented for the [NMPC Book](http://numerik.mathematik.uni-bayreuth.de/~lgruene/nmpc-book/) from Lars Grüne and Jürgen Pannek.

## Installation 

The easiest way to install the **nMPyC** package is using PIP by running 

```
   pip install nmpyc
```

This command will automatically add the package to the Python default path and install the required Python packages and their dependencies which are [CasADi](https://web.casadi.org>), [osqp](https://osqp.org/>), [NumPy](https://numpy.org>), [SciPy](https://scipy.org>), [matplotlib](https://matplotlib.org/stable/index.html) and [dill](https://dill.readthedocs.io/en/latest/dill.html).

***For further informations about the Python library please visit the [nMPyC Documentation](https://nmpyc.readthedocs.io/).***

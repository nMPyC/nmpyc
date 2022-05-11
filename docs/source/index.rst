Welcome to nMPyC's documentation!
===================================

**nMPyC** is a Python library for solving optimal control problems via model predictive control (MPC).

**nMPyC** can be understood as a blackbox method. The user can only enter the desired optimal control problem without having much knowledge of the theory of model predictive control or its implementation in Python. Nevertheless, for an advanced user, there is the possibility to adjust all parameters.

This library supports a variety of discretization methods and optimizers including `CasADi <https://web.casadi.org/>`_ and `SciPy <https://scipy.org/>`_ solvers.

.. note::

   This project is under active development.


Installation
------------

To install the Python package, the source code from `GitLab <https://gitlab.uni-bayreuth.de/bt704963/nmpyc1>`_ has to be downloaded.
This can be done via Git using the command

::

   git clone https://gitlab.uni-bayreuth.de/bt704963/nmpyc1.git

Now the toolbox can be used either by importing the package according to its storage path in the Python code or by adding it to the Python default path.
To realize the letter case you can navigate to the location of the package and use

::

   pip install .

This command will automatically add the package to the Python default path and install the required Python packages and their dependencies which are `CasADi <https://web.casadi.org>`_,  `osqp <https://osqp.org/>`_,  `NumPy <https://numpy.org>`_,  `SciPy <https://scipy.org>`_,  `matplotlib <https://matplotlib.org/stable/index.html>`_ and `dill <https://dill.readthedocs.io/en/latest/dill.html>`_.


Contents
--------

.. toctree::
   :maxdepth: 2

   start
   theory_mpc
   api
   examples
   templates

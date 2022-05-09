Welcome to nMPyC's documentation!
===================================

**nMPyC** is a Python library for solving optimal control problems via model predictive control (MPC).

This Python toolbox should be understood as a blackbox method.
Our goal is to enable the user to enter his problem as easily as possible and to deliver results without having to have much prior knowledge of model predictive control and its implementation in Python.

In addition, our toolbox supports a variety of discretization methods and optimizers including `CasADi <https://web.casadi.org/>`_ and `SciPy <https://scipy.org/>`_ solvers.

.. note::

   This project is under active development.


Installation
------------

To install the Python package, you first need to download the source code from `GitLab <https://gitlab.uni-bayreuth.de/bt704963/nmpyc1>`_.
This can be done via Git using the command

::

   git clone https://gitlab.uni-bayreuth.de/bt704963/nmpyc1.git

Now the tollbox can be used either by importing the package according to its storage path in the python code or by adding it to the python default path.
To realize the letter case you can navigate to the location of the package and use

::

   pip install .

This command will automatically add the package to the python default path and install the required Python packages and their dependencies which are `CasADi <https://web.casadi.org>`_,  `osqp <https://osqp.org/>`_,  `NumPy <https://numpy.org>`_,  `SciPy <https://scipy.org>`_,  `matplotlib <https://matplotlib.org/stable/index.html>`_ and `dill <https://dill.readthedocs.io/en/latest/dill.html>`_.


Contents
--------

.. toctree::
   :maxdepth: 2

   api

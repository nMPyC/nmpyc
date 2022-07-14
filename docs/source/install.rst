Installation
=============


Requirements
^^^^^^^^^^^^^^
The nMPyC package has the following dependencies:

- `CasADi <https://web.casadi.org>`_
- `NumPy <https://numpy.org>`_
- `matplotlib <https://matplotlib.org/stable/index.html>`_ 
- `SciPy <https://scipy.org>`_
- `dill <https://dill.readthedocs.io/en/latest/dill.html>`_
- `osqp <https://osqp.org/>`_


Installation using PIP
^^^^^^^^^^^^^^^^^^^^^^^

The easiest way to install the nMPyC package is to use PIP. To do so, you just need to run the command 

::

   pip install nmpyc

The main advantage of this method is that the package is automatically added to the Python default path and all dependencies are installed. 

Additionally you can update the package by running

::
   
   pip install nmpyc --upgrade


Installation by Source
^^^^^^^^^^^^^^^^^^^^^^^^

To install the Python package by source, the source code from `GitLab <https://gitlab.uni-bayreuth.de/bt704963/nmpyc1>`_ has to be downloaded.
This can be done via Git using the command

::

   git clone https://gitlab.uni-bayreuth.de/bt704963/nmpyc1.git

Now the toolbox can be used either by importing the package according to its storage path in the Python code by adding it to the Python default path.
To realize the letter case you can navigate to the location of the package and use

::

   pip install .

This command will automatically add the package to the Python default path and install the required Python packages and their dependencies.


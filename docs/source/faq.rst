FAQ
====

Why use the `nmpyc_array` module?
------------------------------------------

The idea of the :py:mod:`nmpyc.nmpyc_array` module is to provide a simple syntax for the input, which is similar to the one of NumPy.
At the same time we ensure a switching between symbolic calculation with CasADi and completely numeric calculations. 

A completely numerical calculation is advantageous, for example, if non-differentiable functions have to be evaluated at critical points, e.g. the norm at the origin. Here the algorithmic differentiation of CasADi can lead to problems. 

Therefore this module is built in a way that the array class can automatically switch between CasADi and NumPy objects. In addition, the individual functions are built in such a way that they recognize the type of the input and call the appropriate function from NumPy or CasADi accordingly.


What to do if a function is not defined in the `nmpyc_array` module?
--------------------------------------------------------------------------------

We have tried to implement the most common functions. Nevertheless, it can happen that a certain function that you need is missing.

If this is the case, there is the possibility to implement an own overload of this function. A good orientation for this is the already programmed functions in the :py:mod:`nmpyc.nmpyc_array` module.

Another possibility in such cases is to use the call :code:`x.A` to access the CasADi or NumPy array in which the entries of :code:`x` are stored. Afterwards the appropriate necessary computations can be accomplished with the help of NumPy or CasADi functions. 
Note, however, that in this way if applicable no smooth change between numeric and symbolic calculation is possible. 


Which solver should be used?
---------------------------

In the most cases, the automatic selection of the solver by the program is recommended. In this way, if possible, the linear quadratic structure of a problem is exploited or at least algorithmic differentiation is still exploited to perform an advantageous optimization. 

However, as already mentioned, this algorithmic differentiation can also lead to problems in some cases. For example, if a non-differentiable function must be evaluated at critical points, e.g. the norm at the origin. In such cases, a numerical calculation should be used for the optimization and a SciPy solver, such as SLSQP, should be selected. 


Which discretization method should be used?
------------------------------------------

In our numerical simulations we have experienced that mostly a fixed step integration method like euler is sufficient to guarantee the necessary accuracy during the simulation. The advantage of these methods is that with them the largest speed up among the available integrators can be achieved.

However, if it is necessary to achieve higher integration accuracy by an adaptive integration method, one of the CasADi integrators, e.g. cvodes, should always be chosen if possible.

The SciPy integrators should only be considered as a kind of backup in case the other methods fail, since they lead to an above-average lag of time during the simulation in our implementation. 


What to do if I a LaTeX Error occurs while plotting?
--------------------------------------------------------

In our experience such errors occur mainly on MacOS if Spyder is used for programming, which in turn is opened via the Anaconda Navigator. 
In this case it is sufficient to open spyder directly and not to take the detour via the Anaconda Navigator to solve the problem. 

However, if this procedure does not solve the problem or the problem has another cause, it is also possible to disable the LaTeX labeling of the plots by setting the option :code:`usetex=False`. For more details see :py:meth:`nmpyc.result.result.plot`.


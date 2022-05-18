FAQ
====

Why shoul I use the `nmpyc_array` module?
------------------------------------------

The idea of the :py:mod:`nmpyc.nmpyc_array` module is to provide a simple syntax for the input, which is similar to the one of NumPy.
At the same time we want to ensure a switch between symbolic calculation with CasADi and completely numeric calculations. 

A completely numerical calculation is advantageous, for example, if non-differentiable functions have to be evaluated at critical points. Here the algorithmic differentiation of CasADi can lead to problems. 

Therefore this module is built in a way that the array class can automatically switch between CasADi and NumPy objects. In addition, the individual functions are built in such a way that they recognize the type of the input and call the appropriate function from NumPy or CasADi accordingly.


What should I do, if I need a function not defined in the `nmpyc_array` module?
--------------------------------------------------------------------------------

We have tried to implement the most common functions. Nevertheless, it can happen that a certain function that you need is missing.

If this is the case, experienced users can implement their own overload of this function. A good orientation for this are the already programmed functions in the :py:mod:`nmpyc.nmpyc_array` module.

Another possibility in such a case is to use the call :code:`x.A` to access the CasADi or NumPy array in which the entries of :code:`x` are stored. Afterwards the appropriate necessary computations can be accomplished with the help of NumPy or CasADi functions. 
Note however that in this way if applicable no smooth change between numeric and symbolic calculation is possible. 


Which solver should I use?
---------------------------




Which discretization method should I use?
------------------------------------------


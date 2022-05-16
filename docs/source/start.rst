Getting Started
================

Creating the System Dynmaics
-----------------------------


To define the system dynamics of our problem, we have to create a `nmpyc.system` object.
We can define the maybe time-dependend and nonlinear system dynamics using a function of the following form.

.. code-block:: python

   def f(t,x,u):
       y = nmpyc.array(nx)
       ...
       return y

If this function is created, the system can be initialized by calling 

.. code-block:: python
   
   system = nmpyc.system(f,nx,nu,system_type)

Where `nx` is the dimension of the state, `nu` is the dimension of the control variable, and `system_type` is a string indicating whether the system is continuous (`continuous`) or discrete (`discrete`).


Furthermore, the parameters `sampling_rate` (sampling rate), `t0` (initial time) and `method` can optionally be adjusted during the initialization of the system. The value of `method` determines the used integration method for the discretization of the differential equation in the continuous case. By default the CasADi integrator `cvodes` is used.

Further options of the used integration method can be defined by the call

.. code-block:: python

   system.set_integratorOptions(dict())

For more informations take a look at the API-References :py:class:`nmpyc.system.system`.


Creating the Objective
-----------------------

To define the objective, we need to create -- similar to the system dynamics -- an `nmpyc.objective` object.
To do this, we first define the stage cost:

.. code-block:: python

   def l(t,x,u):
       ...
       return y

Optionally, we can also add terminal costs of the form

.. code-block:: python

   def F(t,x):
       ...
       return y

Now we can initialize the objective by calling

.. code-block:: python

   objective = mpc.objective(l,F)
   #or alternatively without terminal costs
   objective = mpc.objective(l)

For more informations take a look at the API-References :py:class:`nmpyc.objective.objective`.



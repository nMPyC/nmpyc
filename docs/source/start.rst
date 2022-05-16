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

   objective = nmpyc.objective(l, F)
   # Or alternatively without terminal costs
   objective = nmpyc.objective(l)

For more informations take a look at the API-References :py:class:`nmpyc.objective.objective`.

Creating the Constraints
-------------------------

The optimal control problem can be extended with other constraints besides the necessary system dynamics.
To do this, we must first create an empty `mpc.constraints` object using the command 

.. code-block:: python

   system = nmpyc.constraints()

We can now add the desired constraints to this object step by step.
These constraints can be created in different ways.    
First, we can add box constraints in the form of bounds.

.. code-block:: python

constraints.add_bounds('lower', 'control', lbu) # lower bound for control
constraints.add_bounds('upper', 'control', ubu) # upper bound for control

Here `lbu` or `lbx` is an :py:class:`nmpyc.nmpyc_array.array` of dimension `(1,nu)` or `(nu,1)`.    
To add bounds for the state or final state, simply replace `control` with `state` or `terminal` in the above code and adjust the dimension of the array accordingly.

In addition to box constraints, general inequality and equality constraints can also be inserted.

.. code-block:: python

   # Equality constraint h(t,x,u) = 0
   def h(t,x,u):
      y = mpc.array(len_constr)
      ...
      return y
   constraints.add_constr('eq', h) 

   # Inequality constraint g(t,x,u) >= 0
   def g(t,x,u):
      y = mpc.array(len_constr)
      ...
      return y
   constraints.add_constr('ineq', g) 

Terminal constraints of the form :math:`H(t,x) = 0` or :math:`G(t,x) \geq 0` can also be added.

.. code-block:: python

   constraints.add_constr('terminal_eq', H) 
   constraints.add_constr('terminal_ineq', G) 

Moreover it is possible to add linear equality and inequality constraints. 
For this purpose look at the :py:meth:`nmpyc.constraints.constraints.add_constr`.
For further general informations take a look at the API-References :py:class:`nmpyc.oconstraints.constraints`.



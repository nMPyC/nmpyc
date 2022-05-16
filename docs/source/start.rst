Getting Started
================

Import nmpyc
-------------

After the successfull Installation of the nMPyC Package, we have to import nMPyC to use it in our code.
This can be done as shown in the following code snippet 

.. code-block:: python 

   # Add nMPyC to path if necessary
   import sys
   sys.path.append('../../path-to-nmpyc')

   # Import nmpyc
   import nmpyc

Note that the first two lines are omitted if nMPyC has already been added to the Python default path as described in the Installation section. In this case the command `import nmpyc` is sufficient to import nmpyc.

.. note::

   Please use the :py:module:`nmpyc.nmpyc_array` functions and the :py:class:`nmpyc.nmpyc_array.array` class for the calculations in the code to ensure error-free functionality of the program. Further informations about this issue can be found at the API References and the FAQ section.


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
For further general informations take a look at the API-References :py:class:`nmpyc.constraints.constraints`.

Running Simulations
--------------------

After we have initialized all the necessary objects, we can run simulations for our problem. To do this, we first create an `mpc.model` object and combine the different parts of the optimal control problem by calling

.. code-block:: python

   model = nmpyc.model(objective, system, constraints)

The `nmpyc.constraints` object is optional and can be omitted for a problem without constraints.
If we want to adjust the default settings for the optimization, this can be done with the help of the commands 

.. code-block:: python

   model.opti.set_options(dict())
   model.opti.set_solverOptions(dict())

For more informations about this methods look at :py:attr:`nmpyc.model.model.opti`.

Now, to start an open loop simulation, we execute the command

.. code-block:: python

   u_ol, x_ol = model.solve_ocp(x0,N,discount)

and for a closed loop simulation 

.. code-block:: python

   u_ol, x_ol = model.mpc(x0,N,K,discount) 

Here `x0` is an :py:class:`nmpyc.nmpyc_array.array` which defines the initial value, `N` is the MPC horizon and the parameter `K` defines the number of MPC iterations. The parameter `discont` is optional and defines the discount factor if a discounted problem is considered.

The result of the simulation can now be viewed in the console by calling 

.. code-block:: python

   print(res)

and as a visual output by calling 

.. code-block:: python

   res.plot()

By default, the states and controls are displayed in two subplots. By passing a string as the first parameter (`=args`), the graphic can be customized. For example, by calling

.. code-block:: python
   
   res.show('state')

only the states are plotted. Other keywords are `control` for the control, `cost` for the stage costs, and `phase` to make a phase portrait of two states or controls. 
Furthermore, the plots displayed in this way can be additionally adjusted by further prameters, see :py:meth:`nmpyc.result.result.plot`.

Now the model and the simulation results can be saved for later use with the functions

.. code-block:: python

   model.save('path')
   res.save('path')

These saved files can then be loaded with the help of 

.. code-block:: python

   model = nmpyc.model.load('path')
   res = nmpyc.result.load('path')


Advanced topics 
----------------

The above procedure describes only a part of the possibilities of the nMPyC Python library. 
For example, it is also possible to create autonomous systems and use the linear quadratic structure of a problem. 
For further informations to the coding of this problem calsses take a look at the examples and templates section.
And for the implementation of linear system dynamics and quadratic costs, see also :py:meth:`nmpyc.system.system.LQP` and :py:meth:`nmpyc.objective.objective.LQP`.

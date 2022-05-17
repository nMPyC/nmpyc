Autonomous Problem
===================

Time-variant Problem
======================

.. code-block:: Python

   # Import nMPyc package
   import nmpyc 

   # Define system parameters
   nx = .. # dimension of state
   nu = .. # dimension of control
   system_type = .. # system type: continous or discrete 
   sampling_rate = 1. # sampling rate h (optional)
   method = 'cvodes' # integrator (optinal)

   # Define MPC parameters
   N = .. # MPC horizon
   K = .. # MPC itertaions
   x0 = .. # initial value
   discount = 1. # dicount factor (optional)

   # Define right hand side of the system dynamics
   def f(x, u):
      y = nmpyc.array(nx)
      ..
      return y

   # Initialize system dynamics
   system = nmpyc.system(f, nx, nu, system_type, sampling_rate, method=method)

   # Define stage cost
   def l(x, u):
      return ..

   # Define terminal cost (optional)
   def F(x):
      return ..

   # Initialize objective 
   objective = nmpyc.objective(l, F)

   # Define constraints
   constraints = nmpyc.constraints()

   # Add bounds (optional)
   lbx = .. # lower bound for states
   constraints.add_bound('lower', 'state', lbx)
   ubx = .. # upper bound for states
   constraints.add_bound('upper', 'state', ubx)
   lbu = .. # lower bound for control
   constraints.add_bound('lower', 'control', lbu)
   ubu = .. # upper bound for control
   constraints.add_bound('upper', 'control', ubu)
   lbend = .. # lower bound for terminal state
   constraints.add_bound('lower', 'terminal', lbend)
   ubend = .. # upper bound for terminal state
   constraints.add_bound('upper', 'terminal', ubend)

   # Add equality constraints (h(x,u)=0, optional)
   len_eqconstr = .. # number of equality constraints
   def h(x, u):
      c_eq = nmpyc.array(len_eqconstr)
      ..
      return c_eq

   constraints.add_constr('eq', h)

   # Add inequality constraints (g(x,u)>=0, optional)
   len_ineqconstr = .. # number of inequality constraints
   def g(x, u):
      c_ineq = nmpyc.array(len_ineqconstr)
      ..
      return c_ineq

   constraints.add_constr('ineq', g)

   # Add terminal equality constraints (H(x)=0, optional)
   len_terminaleq = .. # number of terminal equality constraints
   def H(x):
      cend_eq = nmpyc.array(len_terminaleq)
      ..
      return cend_eq

   constraints.add_constr('terminal_eq', H)

   # Add terminal equality constraints (G(x)>=0, optional)
   len_terminalineq = .. # number of terminal equality constraints
   def G(x):
      cend_ineq = nmpyc.array(len_terminalineq)
      ..
      return cend_ineq

   constraints.add_constr('terminal_ineq', G)

   # Initialize model
   model = nmpyc.model(objective, system, constraints)

   # Start MPC loop
   res = model.mpc(x0, N, K, discount)

   # Plot results
   res.plot()

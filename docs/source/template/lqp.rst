Linear Quadratic Problem 
=========================

.. code-block:: Python

   # Import nMPyc package
   import nmpyc 

   # Define system parameters
   nx = .. # dimension of state
   nu = .. # dimension of control
   system_type = .. # system type: continuous or discrete 
   sampling_rate = 1. # sampling rate h (optional)
   method = 'cvodes' # integrator (optinal)

   # Define MPC parameters
   N = .. # MPC horizon
   K = .. # MPC itertaions
   x0 = .. # initial value
   discount = 1. # dicount factor (optional)

   # Define linear right hand side of the system dynamics f(x,u) = Ax + Bu
   A = .. 
   B = ..

   # Initialize system dynamics
   system = nmpyc.system.LQP(A, B, nx, nu, system_type, sampling_rate, method=method)

   # Define quadratic stage cost l(x,u) = x^TQx + u^TRu + 2*x^TNx
   Q = ..
   R = ..
   N = nmpyc.zeros((nx,nu)) # optional

   # Define terminal cost x^TPx
   P = nmpyc.zeros((nx,nx)) # optional

   # Initialize objective 
   objective = nmpyc.objective.LQP(Q, R, N, P)

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

   # Add equality constraints (Ex + Fu = b, optional)
   E_eq = ..
   F_eq = ..
   b_eq = ..
   constraints.add_constr('eq', E_eq, F_eq, b_eq)

   # Add equality constraints (Ex + Fu >= b, optional)
   E_ineq = ..
   F_ineq = ..
   b_ineq = ..
   constraints.add_constr('ineq', E_ineq, F_ineq, b_ineq)

   # Add terminal equality constraints (Hx = 0, optional)
   H_eq = ..
   constraints.add_constr('terminal_eq', H_eq)

   # Add terminal equality constraints (Hx >= 0, optional)
   H_ineq = ..
   constraints.add_constr('terminal_ineq', H_ineq)

   # Initialize model
   model = nmpyc.model(objective, system, constraints)

   # Start MPC loop
   res = model.mpc(x0, N, K, discount)

   # Plot results
   res.plot()

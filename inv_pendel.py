#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 11:01:27 2021

@author: Jonas Schiessl
"""

import nmpyc as mpc #import nMPyC package

#---------------------- mpc parameter ---------------------------------------#
nx = 4 # state dimension
nu = 1 # control dimension
system_type = "continuous" #system type: continuous or discrete

h = 0.1 #sampling rate
N = 20 #MPC horizon
K = 100 #MPC iterations
x0 = mpc.array([1, 1, 1, 1])  # initial state 
#----------------------------------------------------------------------------#

######################## NLP #################################################
# In the following lines the implementation of a constrained inverted pendulum
# using the nMPyC package will be described.
# To define the model we have to create a system, objective and 
# (optinal) a constrains object.
# We will define these objects by function which will lead to a nonlinear 
# optimization problem.

#----------------------- system ---------------------------------------------#
# parameters
g = 9.81
k = 0.1

def f(x,u): #alternativ: f(t,x,u) if time dependend
    y = mpc.array(nx)
    y[0] = x[1]
    y[1] = g*x[0] - k*x[1] + u[0]
    y[2] = x[3]
    y[3] = u[0]
    
    return y

system = mpc.system(f, nx, nu, system_type, 
                    sampling_rate=h, method='heun')
#----------------------------------------------------------------------------#

#----------------------- objective ------------------------------------------#
#define stagecosts
def l(x,u): #alternativ: l(t,x,u) if time dependend
    return 2*(x[0]**2 + x[1]**2 + x[2]**2 + x[3]**2) + 4*u[0]**2

objective = mpc.objective(l) #create objective from stagecosts
#----------------------------------------------------------------------------#


#----------------------- constraints ----------------------------------------#
constraints = mpc.constraints() #create empty constraints

# add bounds for control
# (other possible keywords instaed of control are state and terminal)
constraints.add_bound('lower', 'control', mpc.array([-20])) 
constraints.add_bound('upper', 'control', mpc.array([6]))

#define constraints by functions
def g1(x,u): #alternativ: g1(t,x,u) if time dependend
    return x-mpc.array([-9,-9,-9,-9])

def g2(x,u): #alternativ: g2(t,x,u) if time dependend
    return -x+mpc.array([5,5,5,5])

# add inequality constraints
# (equality constraints are also possible by changing 'ineq' to 'eq')
#constraints.add_constr('ineq', g1) #g1(x,u) >= 0
#constraints.add_constr('ineq', g2) #g2(x,u) >= 0
#----------------------------------------------------------------------------#
  
#----------------------- simulation -----------------------------------------#  
model = mpc.model(objective,system,constraints) #create model for simulation

res = model.mpc(x0,N,K) #start MPC loop
res.plot() #plot closed loop results
print()

#change solver to scipy solver SLSQP and change other options
model.opti.set_options(dict(solver='SLSQP', 
                            full_discretization=False, 
                            tol=1e-5, maxiter=2500))
res = model.mpc(x0,5,5) #start simulation (shorter horizon: N=5,K=5)
print(res) #print result
res.show_errors() #show possible errors
print()

#----------------------------------------------------------------------------#


######################## LQP #################################################
# The above problem is a linear quadratic problem (LQP) 
# which means we can also use a linear solver for the problem.
# To do so we have to define the problem as a LQP depending on the different 
# matrices and vectors.
# If system, objective and constraints are defined as linear quadratic 
# the programm will automatically choose the linear solver osqp 
# for optimization (except you change the solver manually).

#----------------------- system ---------------------------------------------#
#define system: x+ = Ax + Bu
A = mpc.array([[0, 1, 0, 0], 
               [g, -k, 0, 0], 
               [0, 0, 0, 1],
               [0, 0, 0, 0]])
B = mpc.array([0, 1, 0, 1])
system = mpc.system.LQP(A, B, nx, nu, system_type, 
                        sampling_rate=h, method='rk4')
#----------------------------------------------------------------------------#

#----------------------- objective ------------------------------------------#
#define stagecosts: l(x,u) = x.TQx + u.TRu (+ 2*x.TNu)  
Q = 2*mpc.eye(nx)
R = 4*mpc.eye(nu)
objective = mpc.objective.LQP(Q, R)
#----------------------------------------------------------------------------#

#----------------------- constraints ----------------------------------------#
constraints = mpc.constraints() #create empty constraints

# add bounds for control
# (other possible keywords instaed of control are state and terminal)
constraints.add_bound('lower', 'control', mpc.array([-20]))
constraints.add_bound('upper', 'control', mpc.array([6]))

#define constraints by matrices and add them to the constraint object
E = mpc.eye(nx)
F = mpc.zeros((nx, nu))
b = mpc.array([-9, -9, -9, -9])
constraints.add_constr('ineq', E, F, b) #Ex +Fu >= b
E = mpc.eye(nx)
F = mpc.zeros((nx,nu))
b = mpc.array([5,5,5,5])
constraints.add_constr('ineq', -E, -F, -b) #-Ex - Fu >= -b <=> Ex + Fu <= b
#----------------------------------------------------------------------------#

#----------------------- simulation -----------------------------------------# 
model = mpc.model(objective, system, constraints)
res = model.mpc(x0, N, K) #start MPC loop
res.show_errors() #show possible errors
res.plot() #plot closed loop results
res.plot('state', show_ol=True) #plot open loop states
res.plot('cost') #plot closed loop stagecosts
#----------------------------------------------------------------------------#
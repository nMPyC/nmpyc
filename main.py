#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 23 15:37:23 2021

@author: Jonas Schiessl
"""
import dill

import numpy as np
import casadi as cas
import nMPyC as mpc
import matplotlib.pyplot as plt
import pickle

nu = 1
nx = 2

def f(t,x,u):
    y = mpc.array(2)
    y[0] = x[0] + u[0]
    y[1] = x[1] + u[0]
    return y

#system = mpc.system(f, nx, nu, system_type='discrete')
system = mpc.system.LQP(mpc.eye(2), mpc.ones((2,1)), 
                        nx, nu, system_type='discrete')
print(system)

def l(t,x,u):
    return x[0]**2 + x[1]**2 + u**2
    
objective = mpc.objective(l)
objective = mpc.objective.LQP(mpc.eye(2),mpc.eye(1))
print(objective)

E = mpc.array([[0,0]])
F = mpc.array([1])
h = mpc.array([-0.5])

T = mpc.eye(2)
he = mpc.array([3,3])

E2 = mpc.eye(2)
F = mpc.array([[0],[0]])
h = 5*mpc.ones(nx)

x0 = mpc.ones(nx)*5

def g(t,x,u): return E2@x + F@u - (h)

constraints = mpc.constraints()
ubu = mpc.ones(nu)*(-1)
constraints.add_bound('lower','control', ubu)
#constraints.add_constr('ineq', E,F,h)
#constraints.add_constr('eq', E2,F,h)
#constraints.add_constr('ineq', E2,F,-h)
#constraints.add_constr('eq', g)
#constraints.add_constr('eq', E2, mpc.array([[0],[0]]),mpc.array([[5],[5]]))
#constraints.add_constr('terminal_eq', T)
#constraints.add_constr('terminal_eq', T)
constraints.add_constr('terminal_ineq', T, -he)
model = mpc.model(objective,system,constraints)

print(constraints)


model.opti.set_options(dict(solver='ipopt', full_discretization=True))
res = model.mpc(x0,5,6)
res.show_errors()
res.plot(usetex = True)

model.opti.set_options(dict(solver='osqp', full_discretization=True))
res = model.mpc(x0,5,6)
res.plot(usetex = True)
res.show_errors()

model.opti.set_options(dict(solver='SLSQP', full_discretization=True))
res = model.mpc(x0,5,6)
res.plot(usetex = True)

# # model.opti.set_options(dict(solver='osqp', full_discretization=False))
# # res = model.mpc(x0,15,5)
# # res.plot(usetex = True)

# res.plot('cost')
# print()
# print(res)
# model.save('untitled.pickle')

# e = mpc.model.load('untitled.pickle')
# e.opti.set_options(dict(solver='SLSQP', full_discretization=True))
# res2 = e.mpc(x0,10,10)
# print(e)
# print(res2)
# res2.plot()

# model.opti.set_options(dict(solver='osqp', full_discretization=True))
# model.mpc(x0,15,5)

# model.opti.set_options(dict(solver='SLSQP', full_discretization=True))
# model.mpc(x0,15,5)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Jonas Schiessl
"""

import sys
sys.path.append('../')

import nmpyc

nu = 1 # dimension of state
nx = 2 # dimension of control

# parameters
V = 10.
cf_A = 1.
cf_B = 0.
k_r = 1.2

def f(t,x,u):
    y = nmpyc.array(2)
    h = 0.5
    y[0] = x[0] + 0.5*((u[0]/V) *(cf_A - x[0]) - k_r*x[0])
    y[1] = x[1] + 0.5*((u[0]/V) *(cf_B - x[1]) + k_r*x[1])
    return y

system = nmpyc.system(f, nx, nu, system_type='discrete')

def l(t,x,u):
    return 0.5 * (x[0]-0.5)**2 + 0.5 * (x[1]-0.5)**2 + 0.5 * (u[0]-12)**2

objective = nmpyc.objective(l)

x0 = nmpyc.array([0.4,0.2])
xeq = nmpyc.array([0.5,0.5])

constraints = nmpyc.constraints()
lbx = nmpyc.zeros(nx)
ubu = nmpyc.ones(nu)*(20)
lbu = nmpyc.zeros(nu)
constraints.add_bound('lower','state', lbx)
constraints.add_bound('lower','control', lbu)
constraints.add_bound('upper','control', ubu)

def he(x): 
    return x - xeq
constraints.add_constr('terminal_eq', he)

model = nmpyc.model(objective,system,constraints)
model.opti.set_options(dict(solver='ipopt', full_discretization=True))
res = model.mpc(x0,15,100)
res.plot()








#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Jonas Schiessl
"""

import sys
sys.path.append('../')

import nmpyc

nx = 4 
nu = 1 
system_type = "continuous" 

h = 0.1 
N = 20
K = 100 
x0 = nmpyc.array([1, 1, 1, 1]) 

g = 9.81
k = 0.1

A = nmpyc.array([[0, 1, 0, 0], 
                 [g, -k, 0, 0], 
                 [0, 0, 0, 1],
                 [0, 0, 0, 0]])
B = nmpyc.array([0, 1, 0, 1])
system = nmpyc.system.LQP(A, B, nx, nu, system_type, 
                        sampling_rate=h, method='rk4')
system.set_integratorOptions(dict(number_of_finit_elements=100))

Q = 2*nmpyc.eye(nx)
R = 4*nmpyc.eye(nu)
objective = nmpyc.objective.LQP(Q, R)

constraints = nmpyc.constraints() 
constraints.add_bound('lower', 'control', nmpyc.array([-20]))
constraints.add_bound('upper', 'control', nmpyc.array([6]))
E = nmpyc.eye(nx)
F = nmpyc.zeros((nx, nu))
b = nmpyc.array([-9, -9, -9, -9])
constraints.add_constr('ineq', E, F, b)
E = nmpyc.eye(nx)
F = nmpyc.zeros((nx,nu))
b = nmpyc.array([5,5,5,5])
constraints.add_constr('ineq', -E, -F, -b)

model = nmpyc.model(objective, system, constraints)
res = model.mpc(x0, N, K)
res.show_errors()
res.plot() 
res.plot('state', show_ol=True) 
res.plot('cost') 
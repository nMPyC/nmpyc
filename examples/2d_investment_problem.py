#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Jonas Schiessl
"""

import sys
sys.path.append('../')

import nmpyc

# parameters
sigma = 0.25
k1 = 2
k2 = 0.0117
c1 = 0.75
c2 = 2.5
alpha = 12

# MPC options
h = 0.2    
N = 50  
K = 500 
system_type = "continuous" 
discont = nmpyc.exp(-0.04*h)

nx = 2  # state dimension
nu = 1  # control dimension

# define ODE
def f(x,u):
    y = nmpyc.array(2)
    y[0] = x[1]-sigma *x[0]
    y[1] = u
    return y

# define stagecosts
def l(x,u):
    R = k1*x[0]**(1/2) - x[0]/(1 + k2*x[0]**4)
    c = c1*x[1] + (c2*x[1]**2)/2
    v = (alpha*u[0]**2)/2
    l_ = -(R - c - v)
    return l_

system = nmpyc.system(f, nx, nu, system_type, sampling_time=h, method='heun')

objective = nmpyc.objective(l)

model = nmpyc.model(objective, system)

x0 = nmpyc.array([3.0,0.75])
res1 = model.mpc(x0,N,K,discont)
res1.plot('phase', phase1='x_1', phase2='x_2', show_ol=True)

x0 = nmpyc.array([5.0,1.75])
res2 = model.mpc(x0,N,K,discont)
res2.plot('phase', phase1='x_1', phase2='x_2', show_ol=True)

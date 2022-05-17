#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 13:58:18 2021

@author: Jonas Schiessl
"""

import nmpyc as mpc 

# parameters
sigma = 0.25
k1 = 2
k2 = 0.0117
c1 = 0.75
c2 = 2.5
alpha = 12

# MPC options
h = 0.2         # sampling rate
N = 50          # MPC horizon length
K = 500          # final time for the MPC loop
system_type = "continuous" 
discont = mpc.exp(-0.04*h)

nx = 2  # state dimension
nu = 1  # control dimension

# define ODE
def f(x,u):
    y = mpc.array(2)
    y[0] = x[1]-sigma *x[0]
    y[1] = u
    return y

# define stagecosts
def l(x,u):
    R = k1*x[0]**(1/2)-x[0]/(1+k2*x[0]**4)
    c = c1*x[1]+(c2*x[1]**2)/2
    v = (alpha*u[0]**2)/2
    l_ = -(R - c - v)
    return l_

system = mpc.system(f, nx, nu, system_type, sampling_rate=h, method='heun')

objective = mpc.objective(l)

model = mpc.model(objective,system)

x0 = mpc.array([3.0,0.75])
res = model.mpc(x0,N,K,discont)
res.plot('phase', phase1='x_1', phase2='x_2', show_ol=True)

x0 = mpc.array([5.0,1.75])
res = model.mpc(x0,N,K,discont)
res.plot('phase', phase1='x_1', phase2='x_2', show_ol=True)
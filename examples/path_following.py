#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Jonas Schiessl
"""

import sys
sys.path.append('../')

import nmpyc

nx = 3 + 1
nu = 2 + 1
system_type = "continuous" 

h = 0.1
N = 10
K = 100
    
# parameters
theta_ = -30
alpha = 6
beta = 5
gamma = 20
omega = 0.35
lambd = -1e-3

u_eq2 = nmpyc.arctan((1 + (-2.1*nmpyc.log(4))**2)**(-3/2)*(-0.84))
u_eq = nmpyc.array([0., u_eq2, 0.])
x_eq = nmpyc.array([0., 0., 0., 0.])

def delta_rho(theta):
    res = -alpha*omega*nmpyc.cos(omega*theta)*nmpyc.log(gamma/(beta - theta))
    res -= alpha/(beta - theta) * nmpyc.sin(omega*theta)
    return res

def path(theta):
    rho = -alpha*nmpyc.log(gamma/(beta + (-theta)))*nmpyc.sin(omega*theta)
    p = nmpyc.array(3)
    p[0] = theta
    p[1] = rho
    p[2] = nmpyc.arctan(delta_rho(theta))
    
    return nmpyc.array(p)

path0 = path(theta_)
x0 = nmpyc.array([path0[0], path0[1], path0[2], theta_])

def f(x,u):
    y = nmpyc.array(nx)
    y[0] = u[0]*nmpyc.cos(x[2])
    y[1] = u[0]*nmpyc.sin(x[2])
    y[2] = u[0]*nmpyc.tan(u[1])
    
    y[3] = x[3]*(-lambd) + u[2]
    
    return y

system = nmpyc.system(f, nx, nu, system_type, sampling_time=h, method='heun')

Q = nmpyc.diag([1e4, 1e5, 1e5])*8
R = nmpyc.diag([10, 10, 1])
eps = 1740

def l(x,u):
    p = path(x[3])
    y = x[:3] - p
    w = (u - u_eq)
    return y.T@Q@y + x[3]**2*8*1/16 + w.T@R@w

def F(x):
    return eps/2*x[3]**2

objective = nmpyc.objective(l,F)

def he(x): 
    return x[:3] - path(x[3])
    
constraints = nmpyc.constraints()
constraints.add_bound('lower','control', nmpyc.array([[0, -0.63, 0]]))
constraints.add_bound('upper','control', nmpyc.array([[6, 0.63, 6]]))
constraints.add_bound('lower','state', nmpyc.array([-nmpyc.inf, -nmpyc.inf, -nmpyc.inf, theta_]))
constraints.add_bound('upper','state', nmpyc.array([nmpyc.inf, nmpyc.inf, nmpyc.inf, 0]))
constraints.add_constr('terminal_eq', he)

model = nmpyc.model(objective, system, constraints)
model.opti.set_options(dict(solver='ipopt', full_discretization=True))
res = model.mpc(x0, N, K)

fig = res.plot(linewidth=3.5)
res.plot('state', xk = [1], show_legend=False)
res.plot('control', show_ol = True)
res.plot('phase', phase1 = 'x_1', phase2 = 'x_2', show_ol = True)
res.plot('cost', show_ol = True)
res.plot('cost')
res.show_errors()

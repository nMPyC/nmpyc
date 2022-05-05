#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 11:01:27 2021

@author: Jonas Schiessl
"""

import nmpyc as mpc

nx = 3+1 # state dimension
nu = 2+1 # control dimension
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

u_eq2 = mpc.arctan((1+(-2.1*mpc.log(4))**2)**(-3/2)*(-0.84))
u_eq = mpc.array([0.,u_eq2,0.])
x_eq = mpc.array([0.,0.,0.,0.])

def delta_rho(theta):
    res = -alpha*omega*mpc.cos(omega*theta)*mpc.log(gamma/(beta-theta))
    res -= alpha/(beta-theta) * mpc.sin(omega*theta)
    return res

def path(theta):
    rho = -alpha*mpc.log(gamma/(beta+(-theta)))*mpc.sin(omega*theta)
    p = mpc.array(3)
    p[0] = theta
    p[1] = rho
    #delta_rho = (alpha*theta*mpc.sin(omega*theta))/(mpc.abs(theta)*(mpc.abs(theta)+beta)) - alpha*omega*mpc.cos(omega*theta)*mpc.log(gamma/(mpc.abs(theta)+beta))
    p[2] = mpc.arctan(delta_rho(theta))
    
    return mpc.array(p)

path0 = path(theta_)
x0 = mpc.array([path0[0],path0[1],path0[2],theta_])  # initial state 

print(path(0))

def f(x,u):
    y = mpc.array(nx)
    y[0] = u[0]*mpc.cos(x[2])
    y[1] = u[0]*mpc.sin(x[2])
    y[2] = u[0]*mpc.tan(u[1])
    
    y[3] = x[3]*(-lambd) + u[2]
    
    return y

system = mpc.system(f, nx, nu, system_type, sampling_rate=h, method='heun')

Q = mpc.diag([1e4,1e5,1e5])*8
R = mpc.diag([10,10,1])
eps = 1740

def l(x,u):
    p = path(x[3])
    y = x[:3] - p
    w = (u-u_eq)
    return y.T@Q@y + x[3]**2*8*1/16 + w.T@R@w

def F(x):
    return eps/2*x[3]**2

objective = mpc.objective(l,F)

def he(x): 
    return x[:3] - path(x[3])
    
constraints = mpc.constraints()

constraints.add_bound('lower','control', mpc.array([[0,-0.63,0]]))
constraints.add_bound('upper','control', mpc.array([[6,0.63,6]]))
constraints.add_bound('lower','state', mpc.array([-mpc.inf,-mpc.inf,-mpc.inf,theta_]))
constraints.add_bound('upper','state', mpc.array([mpc.inf,mpc.inf,mpc.inf,0]))

constraints.add_constr('terminal_eq', he)

model = mpc.model(objective,system,constraints)

model.opti.set_options(dict(solver='ipopt', full_discretization=True))
#model.opti.set_solverOptions(dict(bound_relax_factor = 1e-05))

res = model.mpc(x0,N,K)

fig = res.plot(linewidth=3.5)
res.plot('state', xk = [1], show_legend=False)
res.plot('control', show_ol = True)
res.plot('phase', phase1 = 'x_1', phase2 = 'x_2', show_ol = True)
res.plot('cost', show_ol = True)
res.plot('cost')
res.show_errors()

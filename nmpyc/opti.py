#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl

import casadi as cas
import numpy as np

import osqp

import nmpyc as mpc
from nmpyc.utils import flat_list

from scipy.optimize import minimize
import scipy.sparse as sparse
from scipy.linalg import block_diag
from numpy.linalg import matrix_power

class opti:
    """
    A class used to sore the optimization problem of the OCP.
    
    In this class the choosen optimizer is initialized and the minimization 
    method is called if the problem should bes solved durcing the MPC loop.
    """
    
    def __init__(self):
        
        self._solver = 'auto'
        self._full_discretization = True 
        self._tol = 1e-06
        self._maxiter = 5000
        self._verbose = False
        
        self._U_start = None
        
        self._options={}
    
    @property
    def solver(self):
        """str : Used solver for optimization."""
        return self._method
    
    @solver.setter 
    def solver(self,value):
        if value == 'osqp':
            self._solver = 'osqp'
        elif value in ['sqpmethod','ipopt']:
            self._solver = 'casadi'
        elif value in ['SLSQP', 'trust-constr']:
            self._solver = 'scipy'
        elif value == 'auto':
            self._solver = value
        else: 
            raise ValueError(value + ' is not a availible solver')
            
        self._method = value
        
    @property 
    def full_discretization(self):
        """bool : If true full discretization is applied to the problem."""
        return self._full_discretization
    
    @full_discretization.setter 
    def full_discretization(self, value):
        if isinstance(value, bool):
            self._full_discretization = value
        else:
            raise TypeError(
                'full_discretization must be of type boolean - not ' 
                + str(type(value)))
            
        self._full_discretization = value
            
    @property 
    def tol(self):
        """float : Tolerance of the optimizer."""
        return self._tol
    
    @tol.setter 
    def tol(self, value):
        if not isinstance(value, (int,float)):
            raise TypeError(
                'tol must be of type int or float - not ' 
                + str(type(value)))
        
        if value <= 0:
            raise ValueError(
                'tol must be grater than zero - you have ' 
                + str(value))
            
        self._tol = value
        
    @property 
    def maxiter(self):
        """int : Maximal number of itertaions during the optimization."""
        return self._maxiter
    
    @maxiter.setter
    def maxiter(self, value):
        if isinstance(value, float):
            if not value.is_integer():
                raise TypeError(
                    'maxiter must be of type integer - not ' 
                    + str(type(value)))
            else:
                value = int(value)
                
        if not isinstance(value, int):
            raise TypeError(
                    'maxiter must be of type integer - not ' 
                    + str(type(value)))
            
        if value <= 0:
            raise ValueError(
                'maxiter must be grater than zero - you have ' 
                + str(value))
            
        self._maxiter = value
        
    @property 
    def verbose(self):
        """bool : If true the verbose option of the optimizer is activated."""
        return self._verbose
    
    @verbose.setter 
    def verbose(self, value):
        if isinstance(value, bool):
            self._full_discretization = value
        else:
            raise TypeError(
                'verbose must be of type boolean - not ' 
                + str(type(value)))
            
        self._verbose = value
            
    @property 
    def U_start(self, value):
        """array : Initial guess for the optimizer."""

        if not isinstance(value, array):
            raise TypeError(
                'inital guess must be of type array - not ' 
                + str(type(value)))
        
        if value.symbolic:
            raise TypeError(
                'inital guess mhas to be purely numeric,' 
                + ' but also has symbolic entries.')

        return self._U_start
    
    #TO-DO: proof initial guess
    @U_start.setter 
    def U_start(self,value):
        self._U_start = value
        
    @property 
    def solver_options(self):
        """dict : Dictionary containing the solver dependent options"""
        return self._options
    
    def _delete_optistack(self):
        self._optistack = None
        self._X = None
        self._U = None
        self._p = None
        self._t = None  
        
    def set_options(self, opts):
        """Set the solver independent options.

        Parameters
        ----------
        opts : dict
            Dictionary containing the keywords of the required options 
            and their values.
            
        """
        
        if not isinstance(opts, dict):
            raise TypeError(
                'opts must be of type dictionary - not ' + str(type(opts)))
        
        for key in opts.keys():
            if key in ['solver', 'full_discretization','tol', 
                       'maxiter', 'verbose', 'initial_guess']:
                if key == 'solver': 
                    self.solver = opts['solver']
                elif key == 'full_discretization': 
                    self.full_discretization = opts['full_discretization']
                elif key == 'tol':
                    self.tol = opts['tol']
                elif key == 'maxiter':
                    self.maxiter = opts['maxiter']
                elif key == 'verbose':
                    self.verbose = opts['verbose']
                elif key == 'initial_guess':
                    self.U_start = opts['initial_guess']
            else: 
                raise KeyError(key + ' is not a valid option')
            
    def set_solverOptions(self, options):
        """Set the solver dependent options.

        Parameters
        ----------
        opts : dict
            Dictionary containing the keywords of the required options 
            and their values.
            
        """
        
        if not isinstance(options, dict):
            raise TypeError(
                'opts must be of type dictionary - not ' + str(type(options)))
            
        self._options = options             
        
    def init_solver(self, objective, system, constraints, N):
        """Initalize the solver for the OCP defined by the input Parameters.

        Parameters
        ----------
        objective : objective
            Objective of the optimal control problem.
        system : system
            System of the optimal control problem.
        constraints : constraint
            Constraints of the optiomal control problem.
        N : int
            Prediction horizon for the finit horizon optimal control problem.

        """
        
        if isinstance(objective, mpc.objective):
            self._objective = objective
        else:
            raise TypeError(
                'objective must be of type objective - not ' 
                + str(type(objective)))
        
        if isinstance(system, mpc.system):
            self._system = system
        else:
            raise TypeError(
                'system must be of type system - not ' 
                + str(type(system)))
         
        if isinstance(constraints, mpc.constraints):
            self._constraints = constraints
        else:
            raise TypeError(
                'constraints must be of type constraints - not ' 
                + str(type(constraints)))
        
        if isinstance(N, int):
            if N <= 0 : 
                raise ValueError('MPC horizon N must be greater than zero')
            self._N = N
        else: 
            raise TypeError(
                'MPC horizon N must be of type integer - not ' + str(type(N)))
        
        self._nx = system.nx
        self._nu = system.nu
        
        self._t0 = self._system.t0
        
        if self._constraints.lower_bndx is None:
            self._constraints.add_bound('lower', 'state', 
                                        mpc.ones(self._nx) * (-mpc.inf))
        if self._constraints.upper_bndx is None:
            self._constraints.add_bound('upper', 'state', 
                                        mpc.ones(self._nx) * (mpc.inf))
        if self._constraints.lower_bndu is None:
            self._constraints.add_bound('lower', 'control', 
                                        mpc.ones(self._nu) * (-mpc.inf))
        if self._constraints.upper_bndu is None:
            self._constraints.add_bound('upper', 'control', 
                                        mpc.ones(self._nu) * (mpc.inf))
        if self._constraints.lower_bndend is None:
            self._constraints.add_bound('lower', 'terminal', 
                                        self._constraints.lower_bndx)
        if self._constraints.upper_bndend is None:
            self._constraints.add_bound('upper', 'terminal', 
                                        self._constraints.upper_bndx)
        
        if self._constraints.lower_bndx.dim[0] != self._nx:
            raise ValueError(
                'lower bound for state has the wrong dimension - ' 
                + str(self._constraints.lower_bndx.dim[0]) 
                + '!=' 
                + str(self._nx))
        if self._constraints.upper_bndx.dim[0] != self._nx:
            raise ValueError(
                'upper bound for state has the wrong dimension - ' 
                + str(self._constraints.upper_bndx.dim[0]) 
                + '!=' 
                + str(self._nx))
        if self._constraints.lower_bndu.dim[0] != self._nu:
            raise ValueError(
                'lower bound for control has the wrong dimension - ' 
                + str(self._constraints.lower_bndu.dim[0]) 
                + '!=' 
                + str(self._nu))
        if self._constraints.upper_bndu.dim[0] != self._nu: 
            raise ValueError(
                'upper bound for control has the wrong dimension - ' 
                + str(self._constraints.upper_bndu.dim[0]) 
                + '!=' 
                + str(self._nu))
        if self._constraints.lower_bndend.dim[0] != self._nx:
            raise ValueError(
                'lower bound for terminal state has the wrong dimension - ' 
                + str(self.constraints.lower_bndend.dim[0]) 
                + '!=' 
                + str(self._nx))
        if self._constraints.upper_bndend.dim[0] != self._nx:
            raise ValueError(
                'upper bound for terminal state has the wrong dimension - ' 
                + str(self.constraints.upper_bndend.dim[0]) 
                + '!=' 
                + str(self._nx))
        
        # Set initial guess for the control by default to 0.1
        if self._U_start is None:
            self._U_start = mpc.ones((self._nu, self._N))*0.1
        if (self._U_start.dim[0] != self._nu or 
            self._U_start.dim[1] != self._N):
            self._U_start = mpc.ones((self._nu, self._N))*0.1
            print('Warning: Initial guess has the wrong dimension.' 
                  + ' Changing to default vaulue.')
        self._X_start = mpc.array((self._nx, self._N + 1))
        
        if self._solver == 'auto':
            if (self._system.type == 'LQP' and 
                self._objective.type == 'LQP' and 
                self._constraints.type == 'LQP'):
                self._solver = 'osqp'
                self._method = 'osqp'
            elif self._system._integrator != 'scipy':
                self._solver = 'casadi'
                self._method = 'ipopt'
            else:
                self._solver = 'scipy'
                self._method = 'SLSQP'
            
        if self._solver == 'osqp':
            if (self._system.type == 'NLP' or 
                self._objective.type == 'NLP' or 
                self._constraints.type == 'NLP'):
                raise ValueError('osqp could only be applied to LQPs')
            self._init_osqp()
            
        elif self._solver == 'casadi':
            if self._system._integrator == 'scipy':
                raise ValueError(
                    'casadi could not be used with a scipy-integrator')
            self._init_casadi()
            
        elif self._solver == 'scipy':
            self._init_scipy()
        
        else: 
            raise ValueError(self._solver + 'is not a valid solver')
            
    def _init_osqp(self):
        
        self._A = None
        self._u = None
        self._l = None
        
        Q = mpc.convert(self._objective.stagecost[0],'numpy')
        R = mpc.convert(self._objective.stagecost[1],'numpy')
        NN = mpc.convert(self._objective.stagecost[2],'numpy')
        PP = mpc.convert(self._objective.terminalcost,'numpy')
        
        if self._full_discretization:
            self._A = np.eye(self._nx*(self._N+1)+self._nu*self._N)
            GG = np.block([np.eye(self._nx),
                           np.zeros((self._nx, (self._nu+self._nx)*self._N))])
        
            for i in range(self._N):
                G = np.block([np.zeros((self._nx,(i)*(self._nx+self._nu))),
                              mpc.convert(self._system.f[0]),
                              mpc.convert(self._system.f[1]), 
                              -np.eye(self._nx), 
                              np.zeros((self._nx,(self._N - i - 1)
                                        *(self._nx+self._nu)))])
                GG = np.vstack((GG, G))
            
            A = np.vstack((GG, self._A))
            
            self._l = np.zeros(self._nx*(self._N+1)*2 + self._nu*self._N)
            self._u = np.zeros(self._nx*(self._N+1)*2 + self._nu*self._N)
            for i in range(self._N):
                li = self._nx*(self._N + 1) + (i + 1)*self._nx + i*self._nu
                ui = (self._nx*(self._N + 1) 
                      + (i + 1)*self._nx 
                      + (i + 1)*self._nu)
                self._l[li:ui] = mpc.convert(self._constraints.lower_bndu, 
                                             'numpy').flatten()
                li = self._nx*(self._N + 1) + i*self._nx + i*self._nu
                ui = self._nx*(self._N + 1) + (i + 1)*self._nx + i*self._nu
                self._l[li:ui] = mpc.convert(self._constraints.lower_bndx, 
                                             'numpy').flatten()
            for i in range(self._N):
                li = self._nx*(self._N + 1) + (i + 1)*self._nx + i*self._nu
                ui = (self._nx*(self._N + 1) 
                      + (i + 1)*self._nx 
                      + (i + 1)*self._nu)
                self._u[li:ui] = mpc.convert(self._constraints.upper_bndu, 
                                             'numpy').flatten()
                li = self._nx*(self._N + 1) + i*self._nx + i*self._nu
                ui = self._nx*(self._N + 1) + (i + 1)*self._nx + i*self._nu
                self._u[li:ui] = mpc.convert(self._constraints.upper_bndx, 
                                             'numpy').flatten()
            self._l[-self._nx:] = mpc.convert(self._constraints.lower_bndend, 
                                              'numpy').flatten()
            self._u[-self._nx:] = mpc.convert(self._constraints.upper_bndend, 
                                              'numpy').flatten()
            
            for cons in self._constraints.linear_constr['eq']:
                GG = np.block([mpc.convert(cons[0]),
                               mpc.convert(cons[1]),
                               np.zeros((len(cons[0][:,0]),
                                         ((self._nu + self._nx)*(self._N - 1) 
                                          + self._nx)))])
                self._l = np.hstack((self._l,
                                     mpc.convert(cons[2]).flatten()))
                self._u = np.hstack((self._u,
                                     mpc.convert(cons[2]).flatten()))
                for i in range(self.N - 1):
                    G = np.block([np.zeros((len(cons[0][:,0]),
                                            (i + 1)*(self._nx+self._nu))),
                                  mpc.convert(cons[0]),
                                  mpc.convert(cons[1]),
                                  np.zeros((len(cons[0][:,0]),
                                            (self._nx
                                             +((self._N - i - 2)
                                               *(self._nx + self._nu)))))])
                    GG = np.vstack((GG, G))
                    self._l = np.hstack((self._l, 
                                         mpc.convert(cons[2]).flatten()))
                    self._u = np.hstack((self._u, 
                                         mpc.convert(cons[2]).flatten()))
                    
                A = np.vstack((A, GG))
                
            
            for cons in self._constraints.linear_constr['ineq']:
                GG = np.block([mpc.convert(cons[0]),
                               mpc.convert(cons[1]),
                               np.zeros((len(cons[0][:,0]),
                                         ((self._nu + self._nx)*(self._N - 1)
                                          + self._nx)))])
                self._l = np.hstack((self._l, mpc.convert(cons[2]).flatten()))
                for i in range(self._N - 1):
                    G = np.block([np.zeros((len(cons[0][:,0]),
                                            (i + 1)*(self._nx + self._nu))),
                                  mpc.convert(cons[0]),
                                  mpc.convert(cons[1]),
                                  np.zeros((len(cons[0][:,0]),
                                            (self._nx
                                             + ((self._N - i - 2)
                                               *(self._nx + self._nu)))))])
                    GG = np.vstack((GG, G))
                    self._l = np.hstack((self._l,
                                         mpc.convert(cons[2]).flatten()))
                
                A = np.vstack((A, GG))
                self._u = np.hstack((self._u, np.ones(len(GG[:,0]))*np.inf))
            
            for cons in self._constraints.linear_constr['terminal_eq']:
                GG = np.block([np.zeros((len(cons[0][:,0]),
                                         (self._nu + self._nx)*(self._N))),
                               mpc.convert(cons[0])])
                
                A = np.vstack((A, GG))
                self._l = np.hstack((self._l,
                                     mpc.convert(cons[1]).flatten()))
                self._u = np.hstack((self._u, 
                                     mpc.convert(cons[1]).flatten()))
                
            for cons in self._constraints.linear_constr['terminal_ineq']:
                GG = np.block([np.zeros((len(cons[0][:,0]),
                                         (self._nu + self._nx)*(self._N))),
                               mpc.convert(cons[0])])
                
                A = np.vstack((A,GG))
                self._l = np.hstack((self._l,
                                     mpc.convert(cons[1]).flatten()))
                self._u = np.hstack((self._u,
                                     np.ones(len(GG[:,0]))*np.inf))
            
            self._A = sparse.csc_matrix(A)
            
            P = np.block([[Q,NN],[NN.T,R]])
            P_ = np.copy(P)
            for i in range(self._N - 1):
                P_ = block_diag(P_, P)
            P_ = block_diag(P_, PP)
            self._P = sparse.csc_matrix(P_)
            
        else:
            QQ = np.copy(Q)
            RR = np.copy(R)
            N2 = np.copy(NN)
            for i in range(self._N - 1):
                QQ = block_diag(QQ, Q)
                RR = block_diag(RR, R)
                N2 = block_diag(N2, NN)
            N2 = np.vstack((N2, np.zeros((self._nx, self._nu*(self._N)))))
            QQ = block_diag(QQ, PP)
            
            A = mpc.convert(self._system.f[0], 'numpy')
            B = mpc.convert(self._system.f[1], 'numpy')
            
            G = np.vstack((np.zeros((self._nx, 1)), np.ones((self._nu, 1))))
            H = np.block([[np.eye(self._nx)],
                          [np.zeros((self._nu, self._nx))]])
            
            GG = np.copy(G)
            HH = np.copy(H)
            for i in range(self._N - 1):
                GG = block_diag(GG, G)
                HH = block_diag(HH, H)  
            HH = block_diag(HH, np.eye(self._nx))
            GG = np.block([[GG], [np.zeros((self._nx, self._N*self._nu))]])
            
            AA = np.eye(self._nx)
            BB = np.zeros((self._nx, self._nu*self._N))
            for i in range(self._N):
                AA = np.vstack((AA, matrix_power(A, (i + 1))))
                BBrow = np.copy(B)
                for j in range(i):
                    BBrow = np.block([matrix_power(A, (j + 1))@B, BBrow])
                BBrow = np.block([BBrow, 
                                  np.zeros((self._nx, 
                                            self._nu*(self._N - i - 1)))])
                BB = np.vstack((BB, BBrow))
                
            self._l = np.zeros(self._nx*(self._N + 1) + self._nu*self._N)
            self._u = np.zeros(self._nx*(self._N + 1) + self._nu*self._N)
            for i in range(self._N):
                li = (i + 1)*self._nx + i*self._nu
                ui = (i + 1)*self._nx+(i + 1)*self._nu
                self._l[li:ui] = mpc.convert(self._constraints.lower_bndu, 
                                             'numpy').flatten()
                li = i*self._nx + i*self._nu
                ui = (i + 1)*self._nx + i*self._nu
                self._l[li:ui] = mpc.convert(self._constraints.lower_bndx, 
                                             'numpy').flatten()
            for i in range(self._N):
                li = (i + 1)*self._nx + i*self._nu
                ui = (i + 1)*self._nx + (i + 1)*self._nu
                self._u[li:ui] = mpc.convert(self._constraints.upper_bndu, 
                                             'numpy').flatten()
                li = i*self._nx + i*self._nu
                ui = (i + 1)*self._nx + i*self._nu
                self._u[li:ui] = mpc.convert(self._constraints.upper_bndx, 
                                             'numpy').flatten()
            
            index = self._N*self._nx + self._nu*self._N
            self._l[index:] = mpc.convert(self._constraints.lower_bndend, 
                                          'numpy').flatten()
            self._u[index:] = mpc.convert(self._constraints.upper_bndend, 
                                          'numpy').flatten()
            
            
            for cons in self._constraints.linear_constr['ineq']:
                H = mpc.convert(cons[0], 'numpy')
                G = mpc.convert(cons[1], 'numpy')
                G1 = np.copy(G)
                H1 = np.copy(H)
                self._l = np.hstack((self._l, 
                                     mpc.convert(cons[2],'numpy').flatten()))
                self._u = np.hstack((self._u, 
                                     np.ones(len(cons[2]))*np.inf))
                for i in range(self.N - 1):
                    G1 = block_diag(G1, G)
                    H1 = block_diag(H1, H)
                    self._l = np.hstack((self._l, 
                                         mpc.convert(cons[2],'numpy').flatten()))
                    self._u = np.hstack((self._u, 
                                         np.ones(len(cons[2]))*np.inf))
                H1 = np.block([H1, np.zeros(len(H1[:,0], self._nx))])
                HH = np.vstack((HH, H1))
                GG = np.vstack((GG, G1))
            
            for cons in self._constraints.linear_constr['eq']:
                H = mpc.convert(cons[0], 'numpy')
                G = mpc.convert(cons[1], 'numpy')
                G1 = np.copy(G)
                H1 = np.copy(H)
                self._l = np.hstack((self._l, 
                                     mpc.convert(cons[2],'numpy').flatten()))
                self._u = np.hstack((self._u, 
                                     mpc.convert(cons[2],'numpy').flatten()))
                for i in range(self.N - 1):
                    G1 = block_diag(G1, G)
                    H1 = block_diag(H1, H)
                    self._l = np.hstack((self._l, 
                                         mpc.convert(cons[2], 
                                                     'numpy').flatten()))
                    self._u = np.hstack((self._u, 
                                         mpc.convert(cons[2], 
                                                     'numpy').flatten()))
                H1 = np.block([H1,np.zeros(len(H1[:,0], self._nx))])
                HH = np.vstack((HH ,H1))
                GG = np.vstack((GG, G1))
                
            for cons in self._constraints.linear_constr['terminal_ineq']:
                H1 = np.block([np.zeros((len(cons[1]),
                                         len(HH[0,:]) - self._nx)), 
                               mpc.convert(cons[0], 'numpy')])
                HH = np.block([[HH], [H1]])
                GG = np.block([[GG], [np.zeros((self._nx, self._N*self._nu))]])
                self._l = np.hstack((self._l, 
                                     mpc.convert(cons[1],'numpy').flatten()))
                self._u = np.hstack((self._u, 
                                     np.ones(len(cons[1]))*np.inf))
                
            for cons in self._constraints.linear_constr['terminal_eq']:
                H1 = np.block([np.zeros((len(cons[1]), 
                                         len(HH[0,:]) - self._nx)), 
                               mpc.convert(cons[0], 'numpy')])
                HH = np.block([[HH], [H1]])
                GG = np.block([[GG], 
                               [np.zeros((self._nx, self._N*self._nu))]])
                self._l = np.hstack((self._l, 
                                     mpc.convert(cons[1],'numpy').flatten()))
                self._u = np.hstack((self._u, 
                                     mpc.convert(cons[1],'numpy').flatten()))
                
            self._A = sparse.csc_matrix(GG + HH@BB)
            self._HH = HH
            self._AA = AA
            
            q = AA.T@(QQ@BB + N2)
            self._q = lambda x0 : x0.T@q
            self._P = sparse.csc_matrix(RR + BB.T@QQ@BB + BB.T@N2 + N2.T@BB)

        
        self._lqp = osqp.OSQP()
        
        rho = 0.1
        sigma =  1e-06
        max_iter = self._maxiter
        eps_abs = self._tol
        eps_rel = self._tol
        eps_prim_inf = 1e-04
        eps_dual_inf = 1e-04
        alpha = 1.6
        linsys_solver = 'qdldl'
        delta = 1e-06
        polish = False
        polish_refine_iter = 3
        verbose = self._verbose
        scaled_termination = False
        check_termination = 25
        warm_start = True
        scaling = 10
        adaptive_rho = True
        adaptive_rho_interval = 0
        adaptive_rho_tolerance = 5
        adaptive_rho_fraction = 0.4
        time_limit = 0
        
        for key in self._options.keys():
            locals()[key] = self._options[key]
        
        self._lqp.setup(P = self._P, A = self._A, l = self._l, u = self._u, 
                       rho = rho, sigma = sigma, max_iter = max_iter,
                       eps_abs = eps_abs, eps_rel = eps_rel, 
                       eps_prim_inf = eps_prim_inf, eps_dual_inf = eps_dual_inf,
                       alpha = alpha, linsys_solver = linsys_solver, delta = delta,
                       polish = polish, polish_refine_iter = polish_refine_iter,
                       verbose = verbose, scaled_termination = scaled_termination,
                       check_termination = check_termination, warm_start = warm_start,
                       scaling = scaling, adaptive_rho = adaptive_rho,
                       adaptive_rho_interval = adaptive_rho_interval,
                       adaptive_rho_tolerance = adaptive_rho_tolerance,
                       adaptive_rho_fraction = adaptive_rho_fraction,
                       time_limit = time_limit)
            
    def _init_casadi(self):
        
        self._optistack = cas.Opti() 
        
        self._U = self._optistack.variable(self._nu, self._N)      # for control trajectory   
        self._p = self._optistack.parameter(self._nx)             # parameter for initial value
        self._t = self._optistack.parameter(self._N+1)
        
        if self._full_discretization:
            self._X = self._optistack.variable(self._nx, self._N + 1)     # for state trajectory
            # initial conditions
            self._optistack.subject_to(self._X[:,0]  == self._p)
            if self._system.autonomous:
                # add system dynamics as constrains
                for k in range(self._N):
                    x_next = mpc.convert(self._system.system_discrete(0, self._X[:,k], self._U[:,k]))
                    for i in range(self._nx):
                        self._optistack.subject_to(self._X[i, k + 1] == x_next[i])
                
        else:
            self._X = mpc.array((self._nx, self._N + 1))
            self._X[:,0] = self._p
            for k in range(self._N):
                t = self._t0 + k*self._system.h
                self._X[:,k + 1] = self._system.system_discrete(t, self._X[:,k], self._U[:,k])
        
        # Set objective
        J = self._objective.J(self._t, self._X, self._U, self._N)
        J = mpc.convert(J)
        self._optistack.minimize(J)
        
        # Set bounds
        for i in range(self._nx):
            for j in range(self._N):
                self._optistack.subject_to(
                    self._optistack.bounded(self._constraints.lower_bndx[i], 
                                            self._X[i, j], 
                                            self._constraints.upper_bndx[i])) 
        
        for i in range(self._nu):
            self._optistack.subject_to(
                self._optistack.bounded(self._constraints.lower_bndu[i], 
                                        self._U[i, :], 
                                        self._constraints.upper_bndu[i])) 
        
        for i in range(self._nx):
            self._optistack.subject_to(
                self._optistack.bounded(self._constraints.lower_bndend[i], 
                                        self._X[i,self._N], 
                                        self._constraints.upper_bndend[i]))
        
        #set linear constraints
        for cons in self._constraints.linear_constr['eq']:
            for k in range(self._N):
                cons_value = cons[0]@self._X[:, k] + cons[1]@self._U[:, k]
                for i in range(cons_value.dim[0]):
                    self._optistack.subject_to(cons_value[i] <= cons[2][i])
                    self._optistack.subject_to(cons_value[i] >= cons[2][i])
        for cons in self._constraints.linear_constr['ineq']:
            for k in range(self._N):
                cons_value = cons[0]@self._X[:, k] + cons[1]@self._U[:, k]
                for i in range(cons_value.dim[0]):
                    self._optistack.subject_to(cons_value[i] >= cons[2][i])
                
        for cons in self._constraints.linear_constr['terminal_eq']:
            cons_value = cons[0]@self._X[:, self._N]
            for i in range(cons_value.dim[0]):
                self._optistack.subject_to(cons_value[i] <= cons[1][i])
                self._optistack.subject_to(cons_value[i] >= cons[1][i])
        for cons in self._constraints.linear_constr['terminal_ineq']:
            cons_value = cons[0]@self._X[:, self._N]
            for i in range(cons_value.dim[0]):
                self._optistack.subject_to(cons_value[i] >= cons[1][i])
                
        #set nonlinear constraints
        for cons in self._constraints.nonlinear_constr['eq']:
            for k in range(self._N):
                cons_value = cons(self._t[k],self._X[:, k],self._U[:, k])
                for i in range(cons_value.dim[0]):
                    self._optistack.subject_to(cons_value[i] <= 0)
                    self._optistack.subject_to(cons_value[i] >= 0)
        for cons in self._constraints.nonlinear_constr['ineq']:
            for k in range(self._N):
                cons_value = cons(self._t[k],self._X[:, k],self._U[:, k])
                for i in range(cons_value.dim[0]):
                    self._optistack.subject_to(cons_value[i] >= 0)
                
        for cons in self._constraints.nonlinear_constr['terminal_eq']:
            cons_value = cons(self._t[-1],self._X[:, self._N])
            for i in range(cons_value.dim[0]):
                self._optistack.subject_to(cons_value[i] <= 0)
                self._optistack.subject_to(cons_value[i] >= 0)
        for cons in self._constraints.nonlinear_constr['terminal_ineq']:
            cons_value = cons(self._t[-1],self._X[:, self._N])
            for i in range(cons_value.dim[0]):
                self._optistack.subject_to(cons_value[i] >= 0)
                
        # set the optimizer
        options = {}
        options['max_iter'] = self._maxiter
        if self._method == 'ipopt':
            options['sb'] = 'yes'
            if self._verbose is False:
                options['print_level'] = 0
            options['acceptable_tol'] = self._tol
            options.update(self._options)
            self._optistack.solver("ipopt", 
                                   {'print_time': self._verbose},
                                   options)
        elif self._method == 'sqpmethod':
            if self._verbose is False:
                options['print_time'] = False
                options['print_header'] = False
            options['tol_du'] = self._tol
            options['tol_pr'] = self._tol
            options.update(self._options)
            self._optistack.solver('sqpmethod', options)
        
    def _init_scipy(self):
        self._bnds = ()
        lower_bndu = mpc.convert(self._constraints.lower_bndu, 'numpy')
        upper_bndu = mpc.convert(self._constraints.upper_bndu, 'numpy')
        for i in range(self._N):
            for j in range(self._nu):
                self._bnds += ((lower_bndu[j], upper_bndu[j]),)
                
        if self._full_discretization:
            self._Z_start = np.zeros(self._nu*self._N + self._nx*(self._N + 1))
            self._constraints_scipy = ({'type': 'eq', 
                                        'fun': self._cons_dynamics},)
            
            lower_bndx = mpc.convert(self._constraints.lower_bndx, 'numpy')
            upper_bndx = mpc.convert(self._constraints.upper_bndx, 'numpy')
            for i in range(self._N):
                for j in range(self._nx):
                    self._bnds += ((lower_bndx[j] ,upper_bndx[j]),)
            
            lower_bndend = mpc.convert(self._constraints.lower_bndend, 'numpy')
            upper_bndend = mpc.convert(self._constraints.upper_bndend, 'numpy')
            for j in range(self._nx):       
                self._bnds += ((lower_bndend[j], upper_bndend[j]),)
            
        else:
            self._Z_start = np.zeros(self._nu*self._N)
            self._constraints_scipy = ({'type': 'ineq', 
                                        'fun': self._bndsx_scipy},)
        if (self._constraints.nonlinear_constr['eq'] != [] or 
            self._constraints.linear_constr['eq'] != [] or 
            self._constraints.nonlinear_constr['terminal_eq'] != [] or 
            self._constraints.linear_constr['terminal_eq'] != []):
            self._constraints_scipy += ({'type': 'ineq', 
                                         'fun': self._cons_eq_scipy},)
        if (self._constraints.nonlinear_constr['ineq'] != [] or 
            self._constraints.linear_constr['ineq'] != [] or 
            self._constraints.nonlinear_constr['terminal_ineq'] != [] or 
            self._constraints.linear_constr['terminal_ineq'] != []):
            self._constraints_scipy += ({'type': 'ineq', 
                                         'fun': self._cons_ineq_scipy},)
                    
    def _J_scipy(self, Z):
        t = np.arange(self._t0, 
                      self._t0 + (self._N+1)*self._system.h, 
                      self._system.h)
        U = np.reshape(Z[:self._N*self._nu], (self._nu, self._N))
        if self._full_discretization:
            X = np.reshape(Z[self._N*self._nu:], (self._nx, self._N + 1))
        else:
            X = np.zeros((self._nx, self._N + 1))
            X[:, 0] = self._x0
            for k in range(self._N):
                X[:, k + 1] = mpc.convert(
                    self._system.system_discrete(t[k], X[:, k], U[:, k]),
                    'numpy').flatten()
        J = 0.
        if self._objective._type == 'NLP':
            for k in range(self._N):
                J += self._objective._L(t[k],X[:, k],U[:, k])
        else:
            for k in range(self._N):
                J += (X[:, k].T@self._objective._L[0].A@X[:, k]
                      + U[:, k].T@self._objective._L[1].A@U[:, k]
                      + X[:, k].T@self._objective._L[2].A@U[:, k]*2)
                
        if self._objective._F is not None:
            if self._objective._type == 'NLP':
                J += self._objective._F(t[:,self._N], X[:, self._N])
            else:
                J += X[:, self._N].T@self._objective._F.A@X[:, self._N]
        
        return J
    
    def _cons_dynamics(self, Z):
        U = np.reshape(Z[:self._N*self._nu], (self._nu,self._N))
        X = np.reshape(Z[self._N*self._nu:], (self._nx,self._N + 1))
        X_next = np.zeros((self._nx, self._N + 1))
        X_next[:, 0] = self._x0
        for k in range(self._N):
            t = self._t0 + k*self._system.h
            X_next[:, k + 1] = mpc.convert(
                self._system.system_discrete(t, X[:,k], U[:,k]),
                'numpy').flatten()
        return (X_next-X).flatten()
    
    def _cons_eq_scipy(self, Z):
        t = np.arange(self._t0, 
                      self._t0 + self.N*self._system.h, 
                      self._system.h)
        U = np.reshape(Z[:self._N*self._nu], (self._nu, self._N))
        if self._full_discretization:
            X = np.reshape(Z[self._N*self._nu:], (self._nx, self._N + 1))
        else:
            X = np.zeros((self._nx, self._N + 1))
            X[:, 0] = self._x0
            for k in range(self._N):
                X[:, k + 1] = mpc.convert(
                    self._system.system_discrete(t[k], X[:, k], U[:, k]),
                    'numpy').flatten()
        c = []
        
        for cons in self._constraints.linear_constr['eq']:
            for k in range(self._N):
                c += [cons[0]@X[:,k] + cons[1]@U[:, k] - cons[2]]
                c += [-c[-1]]
        
        for cons in self._constraints.linear_constr['terminal_eq']:
            c += [cons[0]@X[:, self._N] - cons[1]]
            c += [-c[-1]]
        
        for cons in self._constraints.nonlinear_constr['eq']:
            for k in range(self._N):
                c += [cons(t[k], X[:, k], U[:, k])]
                c += [-c[-1]]
                
        for cons in self._constraints.nonlinear_constr['terminal_eq']:
            c += [cons(t[-1],X[:,self._N])]
            c += [-c[-1]]
        
        c = flat_list(c)
        return np.array(c).flatten()
    
    def _cons_ineq_scipy(self, Z):
        t = np.arange(self._t0, 
                      self._t0 + self._N*self._system.h, 
                      self._system.h)
        U = np.reshape(Z[:self._N*self._nu], (self._nu, self._N))
        if self._full_discretization:
            X = np.reshape(Z[self._N*self._nu:], (self._nx, self._N + 1))
        else:
            X = np.zeros((self._nx, self._N + 1))
            X[:, 0] = self._x0
            for k in range(self._N):
                X[:, k + 1] = mpc.convert(
                    self._system.system_discrete(t[k], X[:,k], U[:,k]),
                    'numpy').flatten()
        c = []
        
        for cons in self._constraints.linear_constr['ineq']:
            for k in range(self._N):
                c += [cons[0]@X[:, k] + cons[1]@U[:, k] - cons[2]]
        
        for cons in self._constraints.linear_constr['terminal_ineq']:
            c += [cons[0]@X[:, self._N] - cons[1]]
        
        for cons in self._constraints.nonlinear_constr['ineq']:
            for k in range(self._N):
                c += [cons(t[k], X[:, k], U[:, k])]
                
        for cons in self._constraints.nonlinear_constr['terminal_ineq']:
            c += [cons(t[-1], X[:, self._N])]
        
        c = flat_list(c)
        return np.array(c).flatten()
        
    
    def _bndsx_scipy(self,Z):
        U = np.reshape(Z[:self._N*self._nu], (self._nu, self._N))
        X = np.zeros((self._nx, self._N + 1))
        X[:,0] = self._x0
        for k in range(self._N):
            t = self._t0 + k*self._system.h
            X[:, k + 1] = mpc.convert(
                self._system.system_discrete(t, X[:, k], U[:, k]),
                'numpy').flatten()
        c = np.zeros(2*self._nx*(self._N + 1))
        for i in range(self._N):
            for j in range(self._nx):
                if not self._constraints.upper_bndx[j] == mpc.inf:
                    c[i*self._nx + j] = self._constraints.upper_bndx[j] - X[j, i] 
                if not self._constraints.lower_bndx[j] == -mpc.inf:
                    c[self._nx*(self._N + 1) + i*self._nx + j] = -(self._constraints.lower_bndx[j] - X[j, i])
        for j in range(self._nx):
            if not self._constraints.upper_bndx[j] == mpc.inf:
                c[self._N*self._nx + j] = self._constraints.upper_bndend[j] - X[j, i] 
            if not self._constraints.lower_bndx[j] == -mpc.inf:
                c[self._nx*(self._N + 1)+ self._N*self._nx + j] = -(self._constraints.lower_bndend[j] - X[j, i])
        return c
            
    def solve(self, t, x0):
        """Start the optimization progress.

        Parameters
        ----------
        t : float
            Current time.
        x0 : array
            Current state.

        Returns
        -------
        array
            Optimal control sequence.
        array
            Optimal trajectory.

        """
        
        if not isinstance(t, (float,int)):
            raise TypeError('time t must be of type integer or float')
            
        if isinstance(x0, mpc.array):
            if x0.dim != (self._nx, 1):
                raise ValueError('x0 has the wrong dimension - ' + str(x0.dim) + ' != (' + str(self._nx) + ',1)')
        else: raise TypeError('x0 must be of type array - not ' + str(type(x0)))
        
        self._t0 = t
        self._X_start[:, 0] = x0
        self._x0 = mpc.convert(x0, 'numpy').flatten()
        
        #calculate the state fitting to the initial guess for the control (U_start)
        if self._full_discretization:
            for k in range(self._N):
                t = self._t0 + k*self._system.h
                self._X_start[:, k + 1] = self._system.system_discrete(
                    t, self._X_start[:, k], self._U_start[:, k])
                
        if self._solver == 'osqp':
            if self._full_discretization:
                self._l[:self._nx] = self._x0
                self._u[:self._nx] = self._x0
            
                self._lqp.update(l=self._l, u=self._u)
            else:
                q = self._q(self._x0)
                l = self._l - self._HH@self._AA@self._x0
                u = self._u - self._HH@self._AA@self._x0
                self._lqp.update(q=q, l=l, u=u)
            
            res = self._lqp.solve()
            if res.info.status_val != 1:
                return (None, res.info.status)
            
            UU = mpc.array((self._nu,self._N))
            XX = mpc.array((self._nx,self._N+1))
            
            for i in range(self._N):
                if self._full_discretization:
                    UU[:, i] = res.x[(i + 1)*self._nx + i*self._nu:
                                    (i + 1)*self._nx + (i + 1)*self._nu]
                    XX[:, i] = res.x[i*self._nx+i*self._nu:
                                    (i + 1)*self._nx+i*self._nu]
                else:
                    UU[:, i] = res.x[i*self._nu:(i + 1)*self._nu]
                    XX[:, 0] = self._x0
                    for k in range(self._N):
                        t = self._t0 + (k + 1)*self._system.h
                        XX[:, k + 1] = self._system.system_discrete(
                            t, XX[:, k], UU[:, k])
          
        elif self._solver == 'casadi':
            t = np.arange(self._t0, 
                          (self._t0 
                           + (self._N+1)*self._system.h 
                           - (self._system.h/2)), 
                          self._system.h)
            
            if self._system.autonomous:
                # set x0 for optimization
                self._optistack.set_value(self._p, self._x0)
            else:
                self._init_casadi()
                # set x0 for optimization
                self._optistack.set_value(self._p, self._x0)
                # add system dynamics as constrains
                for k in range(self._N):
                    x_next = self._system.system_discrete(
                        t[k], self._X[:, k], self._U[:, k])
                    for i in range(self._nx):
                        self._optistack.subject_to(
                            self._X[i, k + 1] == x_next[i])
                        
            # set timespan
            self._optistack.set_value(self._t, t)
            # set initial guess for optimization 
            self._optistack.set_initial(self._U, mpc.convert(self._U_start.A,'numpy')) 
            if self._full_discretization:
                self._optistack.set_initial(self._X, mpc.convert(self._X_start.A,'numpy')) 
            
            try:
                sol = self._optistack.solve() # solve ocp
            except Exception as error:
                return (None,error)
            
            # save control from the current solution as initial guess for the next interation step
            U = np.reshape(sol.value(self._U), (self._nu, self._N))
            if self._full_discretization:
                X = np.reshape(sol.value(self._X), (self._nx, self._N + 1))
            else:
                for k in range(self._N):
                    if self._system.autonomous:
                        t = 0.
                    else:
                        t = self._t0 + k*self._system.h
                    self._X_start[:, k + 1] = self._system.system_discrete(
                        t, self._X_start[:, k], U[:, k])
                X = self._X_start
                
            #self._U_start.copy(U)
            self.U_start = mpc.array(U)
            
            UU = mpc.array(U)
            XX = mpc.array(X)
            
        elif self._solver == 'scipy':
            
            self._Z_start[:self._N*self._nu] = mpc.convert(
                self._U_start, 'numpy').flatten()
            
            if self._full_discretization:
                self._Z_start[self._N*self._nu:] = mpc.convert(
                    self._X_start, 'numpy').flatten()
                
            options = {}
            options['maxiter'] = self._maxiter
            options['disp'] = self._verbose
            options.update(self._options)
            
            res = minimize(self._J_scipy,self._Z_start, 
                           constraints=self._constraints_scipy, 
                           bounds=self._bnds, tol=self._tol, 
                           method=self._method, options=self._options)
            
            if res.success == False: 
                error = res.message
                return (None, error)
            
            U = np.reshape(res.x[:self._nu*self._N], 
                           (self._nu, self._N))
            if self._full_discretization:
                X = np.reshape(res.x[self._nu*self._N:], 
                               (self._nx, self._N + 1))
            else:
                for k in range(self._N):
                    t = self._t0+ k*self._system.h
                    self._X_start[:, k + 1] = self._system.system_discrete(
                        t, self._X_start[:, k], U[:, k])
                X = self._X_start
                
            self._U_start = mpc.array(U)
            
            UU = mpc.array(U)
            XX = mpc.array(X)
            
        return (UU, XX)

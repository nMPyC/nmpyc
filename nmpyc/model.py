#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl

import nmpyc as mpc
from nmpyc.opti import opti

import time
import dill

class model:
    """
    A class that contains all the components of the optimal control problem.
    
    Can be used to perform open and closed loop simulations. 
    
    Parameters
    ----------
    objective : objective
        nMPyC-objective defining the objective of the 
        optimal control problem.
    system : system
        nMPyC-system defining the systemdynamics of the 
        optimal control problem.
    constraints : constraints optional
        nMPyC-constraints defining the constraints of the optimal control 
        problem. If constraints is None the problem is unconstrained. 
        The default is None.

    """
    
    def __init__(self, objective, system, constraints=None):
        
        
        self.objective = objective
        self.system = system   
        self.constraints = constraints
        self._opti = opti()
        self._N = None
        
    @property 
    def objective(self):
        """objective : Objective of the optimal control problem."""
        return self._objective
        
    @objective.setter
    def objective(self,obj):
        if isinstance(obj, mpc.objective):
            self._objective = obj
        else:
            raise TypeError(
                'objective must be of type mpc.objective - not ' 
                + str(type(obj)))
            
    @property
    def system(self):
        """system : Systemdynamic of the optimal control problem."""
        return self._system
    
    @system.setter
    def system(self,syst):
        if isinstance(syst, mpc.system):
            self._system = syst
            self._nx = self._system.nx
            self._nu = self._system.nu
        else:
            raise TypeError(
                'system must be of type mpc.system - not ' 
                + str(type(syst)))
            
    @property
    def constraints(self):
        """constraints : Constraints of the optimal control problem."""
        return self._constraints
    
    @constraints.setter
    def constraints(self,cons):
        if cons is None:
            cons = mpc.constraints()
        if isinstance(cons, mpc.constraints):
            self._constraints = cons
        else:
            raise TypeError(
                'constraints must be of type mpc.constraints - not ' 
                + str(type(cons)))
            
    @property
    def nx(self):
        """int : Dimesnion of the state."""
        return self._nx
    
    @property
    def nu(self):
        """int : Dimension of the control"""
        return self._nu
    
    @property
    def opti(self):
        """opti : Optimizer for the optimal control problem."""
        return self._opti
    
    @property
    def N(self):
        """int : Prediction horizon for the MPC loop."""
        return self._N
    
    @N.setter
    def N(self,NN):
        if isinstance(NN, int):
            if NN <= 0 : 
                raise ValueError('MPC horizon N must be greater than zero')
            self._N = NN
        else: 
            raise TypeError(
                'MPC horizon N must be of type integer - not ' 
                + str(type(NN)))
        
    @property
    def discount(self):
        """float : Discountfactor of the objective"""
        return self._objective.discont
        
    def __str__(self):
        
        string = 'OBJECTIVE: \n'
        string += str(self.objective) + '\n'
        string += 'SYSTEM: \n'
        string += str(self.system) + '\n'
        string += 'CONSTRAINTS: \n'
        string += str(self.constraints)
        
        return string
    
    def solve_ocp(self, x0, N, discount=None):
        """Solving the finit horizon optimal control problem.

        Parameters
        ----------
        x0 : array
            Initial value of the optimal control problem.
        N : int
            Prediction horizon for the control problem.
        discount : float, optional
            Discountfactor of the objective. The default is None.

        Returns
        -------
        u_ol : array
            Optimal control sequence.
        x_ol : array
            Optimal trajectory corresponding to the optimal control sequence.

        """
        self.N = N
        
        self._objective.discont = discount
        
        if isinstance(x0, mpc.array):
            if x0.dim != (self.nx,1):
                raise ValueError(
                    'x0 has the wrong dimension - ' 
                    + str(x0.dim) 
                    + ' != (' + str(self.nx) 
                    + ',1)')
        else: 
            raise TypeError(
                'x0 must be of type mpc.array - not ' 
                + str(type(x0)))
        
        self.opti.init_solver(self.objective, self.system.system_discrete, 
                              self.constraints, self.N)
        
        u_ol, x_ol = self.opti.solve(x0)
        
        return (u_ol, x_ol)
            
    def mpc(self, x0, N, K, discount=None):
        """Solving the optimal control problem with model predictive control.

        Parameters
        ----------
        x0 : array
            Initial state of the optimal control problem.
        N : int
            MPC horizon.
        K : int
            Number of MPC itertaions.
        discount : float, optional
            Discountfactor of the objective. The default is None.

        Returns
        -------
        res : result
            nMPyC result objective containing the optimiaztion results of 
            the closed and open loop simulations.

        """
        
        self.N = N
        self._objective.discont = discount
        
        if isinstance(K, int):
            if K <= 0 : 
                raise ValueError(
                    'Number of Iterations K must be greater than zero')
        else: 
            raise TypeError(
                'Number of Iterations K must be of type integer - not ' 
                + str(type(K)))
        
        if isinstance(x0, mpc.array):
            if x0.dim != (self.nx, 1):
                raise ValueError(
                    'x0 has the wrong dimension - ' 
                    + str(x0.dim) + ' != (' + str(self.nx) + ',1)')
            x = x0
        else: 
            raise TypeError(
                'x0 must be of type mpc.array - not ' 
                + str(type(x0)))
        
        k = 0   # set counter to zero
        
        res = mpc.result(x0, self.system.t0, self.system.h, N, K)
        
        print('Initialize solver ...')
        self.opti.init_solver(self.objective, self.system, 
                              self.constraints, self.N)
        print('... DONE')
        
        print("STARTING MPC-LOOP: (initial value: " + str(x0.flatten()) + ")", 
              flush = True)

        # --- MPC loop ---

        t_s_total = time.time() #timepoint for total time messurement
        
        t = self.system.t0

        while k < K:
            # solve MPC optimal control problem
            t_s_step = time.time() 
            
            # solve ocp
            u_ol, x_ol = self.opti.solve(t, x)
            
            #proof if optimizer terminated sucessfully
            if u_ol is None:
                res._success = False
                res._error = x_ol
                res._K = k
                print('ITERATION ENDED UNSUCCESSFULLY!')
                break

            # get control to the right shape for getting MPC feedback
            u = u_ol[:,0]

            # apply MPC feedback to the system
            x = self.system.system_discrete(t, x, u)
            
            #calculate costs
            l_ol = []
            t_l = t
            for l in range(self.N):
                t_l += self.system.h
                l_ol += [float(self.objective.stagecosts(t_l, x_ol[:,l], u_ol[:,l]))]
            l_ol = mpc.array(l_ol)
            
            #save current itertaion
            res._add_iteration(x_ol, u_ol, l_ol)
            
            # calculate time ellapsed in the iteration step
            elapsed_step = time.time() - t_s_step
            res._time_ol += [elapsed_step]
            
            print("... finished iteration " 
                  + str(k+1) 
                  +" of "
                  + str(K)
                  + " (time: " + str(elapsed_step) + ")", flush = True)
            print("   ... feedback: " + str(u.flatten()), flush = True)
            print("   ... closed loop state: " + str(x.flatten()), 
                  flush = True)
            
            k += 1 # next itertion
            t += self.system.h # next timestep
            
        # calculate total time ellapsed
        elapsed_total = time.time() - t_s_total
        res._time = elapsed_total
        
        res._solver = self._opti.solver
        
        print("END" + " (total time: " + str(elapsed_total) + ")", 
              flush = True)
        
        res._init_cl(t, x)
        
        return res
    
    def save(self, path):
        """Saving the model to a given file with dill.
        
        The path can be absolut or relative and 
        the ending of the file is arbitrary.

        Parameters
        ----------
        path : str
            String defining the path to the desired file. 

        """
        
        self._opti._delete_optistack()
        with open(path, "wb") as output_file:
            dill.dump(self, output_file, -1)
    
    @classmethod
    def load(cls, path):
        """Loads a nMPyC model object from a given path."""
        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load model from file. File not readable!')
            
        if not isinstance(e, model):
            raise Exception(
                'Can not load model from file. File does not cotain a model!')
            
        return e
            
        

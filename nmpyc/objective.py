#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 26 12:58:20 2021

@author: Jonas Schiessl
"""

import nmpyc as mpc
from nmpyc.utilis import mpc_convert

import numpy as np

from inspect import signature
import dill

class objective:
    """
    A class used to define the objective of the optimnal control problem.
    
    The objective depends on stagecosts and optional terminalcosts.
    """
    
    def __init__(self, stagecost, terminalcost=None):
        """
        Parameters
        ----------
        stagecost : callable
            A function defining the stagecosts of the optimal control problem. 
            Has to be of the form l(t,x,u) or l(x,u) in the autonomous case.
        terminalcost : callable, optional
            A function defining the terminalcosts of the optimal control 
            problem. Has to be of the form F(t,x) or F(x) in the autonomous 
            case. If None the no terminalcosts are applied. 
            The default is None.
        """
        
        self.stagecost = stagecost
        self.terminalcost = terminalcost
        self._type = 'NLP'
        self._discont = 1.
        
    @property 
    def stagecost(self):
        """callable : Function defining the stagecosts l(t,x,u)."""
        return self._L
    
    @stagecost.setter 
    def stagecost(self, cost):
        if callable(cost):
            sig = signature(cost)
            params = sig.parameters
            if len(params) == 2:
                self._autonomous = True
                L = lambda t,x,u : cost(x,u)
            elif len(params) == 3:
                self._autonomous = False
                L = cost
            else:
                raise ValueError(
                    'stagecost must have two or three input arguments - not ' 
                    + str(len(params)))
        elif isinstance(cost, list):
            for i in range(3):
                if not isinstance(cost[i], mpc.array): 
                    raise TypeError(
                        'stagecost must be a list of arrays -  not ' 
                        + str(type(cost[i])))
            self._autonomous = True
            L = cost
        else: 
            raise TypeError(
                'stagecost must be callable or list of arrays - not ' 
                + str(type(cost)))
        self._L = L
        
    @property 
    def terminalcost(self):
        """callable : Function defining the endcosts F(t,x)."""
        return self._F
    
    @terminalcost.setter 
    def terminalcost(self, cost):
        F = None
        if callable(cost):
            sig = signature(cost)
            params = sig.parameters
            if len(params) == 1:
                F = lambda t,x : cost(x)
            elif len(params) == 2:
                self.autonomous = False
                F = cost
            else:
                raise ValueError(
                    'terminalcost must have two or three input arguments - not ' 
                    + str(len(params)))
        elif isinstance(cost,mpc.array):
            F = cost
        else: 
            if cost is not None:
                raise TypeError(
                    'terminalcost must be callable or arrays - not ' 
                    + str(type(cost)))
        self._F = F
        
    @property 
    def autonomous(self):
        """bool : If true, the objective is timeindependend."""
        return self._autonomous
    
    @property 
    def type(self):
        """str : If LQP, the objective is quadratic."""
        return self._type
    
    @property
    def discount(self):
        """float : The discountfactor of the objective."""
        return self._discont
    
    @discount.setter
    def discount(self,disc):
        if disc is None:
            disc = 1.
        if isinstance(disc, (int,float)):
            if disc <= 0 or disc > 1.: 
                raise ValueError('discount factor must be in (0,1]')
            self._discont = disc
        else: 
            raise TypeError(
                'discount factor must be of type integer or float - not ' 
                + str(type(disc)))  
        
    def __str__(self):
        
        string = ''
        
        string += 'autonomous: ' + str(self.autonomous) + '\n'
        
        terminal = False
        if self._F is not None: terminal = True
        string += 'terminalcosts: ' + str(terminal) + '\n'
        
        lqp = False
        if self._type == 'LQP': lqp = True
        string += 'linear: ' + str(lqp) + '\n'
        
        string += 'discountfactor: ' + str(self.discount)
        
        return string
        
    @classmethod
    def LQP(cls, Q, R, N=None, P=None):
        """Initialize a linear objective to define a linear quadratic OCP.

        Parameters
        ----------
        Q : array
            Matrix defining the costs of the state of the form x^TQx.
        R : array
            Matrix defining the cost of the control of the form u^TRu.
        N : array, optional
            Possible Matrix defining the mixed costterm of the form 2x^TNu. 
            The default is None.
        P : array, optional
            Posible Matrix defining the endcost of the form x^TPx. 
            The default is None.

        Returns
        -------
        QP : objective
            nMPyC-objective class object suitable to define a linear 
            quadratic optimal control problem.

        """
        
        if N is None:
            N = mpc.zeros((Q.dim[0], R.dim[1]))
        if P is None:
            P = mpc.zeros((Q.dim[0], Q.dim[1]))
            
        if not isinstance(Q, mpc.array):
            raise TypeError('Q must be of type array - not ' + str(type(Q)))
        if not isinstance(R, mpc.array):
            raise TypeError('R must be of type array - not ' + str(type(R)))
        if isinstance(N, mpc.array):
            if N.dim != (Q.dim[0], R.dim[1]):
                raise ValueError(
                    'N has the flase dimension - ' 
                    + str(N.dim) + '!=' + str((Q.dim[0],R.dim[1])))
        else:
            raise TypeError('N must be of type array - not ' + str(type(N)))
        if isinstance(P, mpc.array):
            if P.dim != (Q.dim[0], Q.dim[1]):
                raise ValueError(
                    'N has the flase dimension - ' 
                    + str(P.dim) + '!=' + str((Q.dim[0], Q.dim[1])))
        else:
            raise TypeError('P must be of type array - not ' + str(type(P)))
        
        if Q.symbolic:
            raise ValueError(
                'A has to be purely numeric, but also has symbolic entries')   
        if R.symbolic:
            raise ValueError(
                'R has to be purely numeric, but also has symbolic entries')
        if N.symbolic:
            raise ValueError(
                'N has to be purely numeric, but also has symbolic entries')
        if P.symbolic:
            raise ValueError(
                'P has to be purely numeric, but also has symbolic entries')
        
        QP = cls([Q,R,N], P)
        QP._type = 'LQP'
        return QP
        
    def add_termianlcost(self, terminalcost):
        """Add endcosts to the objective.
        
        The encosts must be a callable function of the form F(t,x) 
        or F(x) in the autonomous case. If endcosts already exists they 
        will be overwritten by the new one given.

        Parameters
        ----------
        terminalcost : callable
            A function defining the terminalcosts of the optimal control 
            problem. Has to be of the form F(t,x) or F(x) in the autonomous 
            case.

        """
        
        self.terminalcost = terminalcost
        
    def J(self, t, x, u, N):
        """Objectivefunction assembeld from the given 
        stagecost and possible endcosts.

        Parameters
        ----------
        t : array
            Timesequence at which the stagecosts and endcosts are evaluated.
        x : array
            State trajectory at which the stagecosts and endcosts are 
            evaluated.
        u : array
            Control sequence at which the stagecosts are evaluated.
        N : int
            Maximum index up to which the stageccosts are to be summed up. 
            During the MPC iteration this index is equivalent to 
            the MPC horizon.

        Returns
        -------
        J : array
            Value of the objectivefunction at the given input parameters.

        """
        
        if isinstance(N, int):
            if N <= 0 : 
                raise ValueError('horizon N must be greater than zero')
        else: 
            raise TypeError(
                'horizon N must be of type integer - not ' + str(type(N)))
        
        J = mpc.array([0])
        for k in range(N):
            J += self.stagecosts(t[k], x[:,k], u[:,k])*(self._discont**(k))
        J += self.endcosts(t[N],x[:,N])
        return J
    
    @mpc_convert
    def stagecosts(self,t,x,u):
        """Stagecosts of the objective.

        Parameters
        ----------
        t : float
            Time at which the stagecost should be evaluated.
        x : array
            Current state at which the stagecost should be evaluated.
        u : array
            Current control at which the stagecost should be evaluated.

        Returns
        -------
        array
            Stagecost evaluated at the given values.
            
        """
            
        if self._type == 'NLP':
            return self._L(t,x,u)
        else:
            return (x.transpose()@self._L[0]@x 
                    + u.transpose()@self._L[1]@u 
                    + x.transpose()@self._L[2]@u*2)

    
    @mpc_convert
    def endcosts(self, t, x):
        """Endcosts of the objective.

        Parameters
        ----------
        t : float
            Time at which the endcost should be evaluated.
        x : array
            Current state at which the endcost should be evaluated.

        Returns
        -------
        array
            Endcost evaluated at the given values.

        """
        
        if self._F is None:
            return mpc.zeros(1)
        if self._type == 'NLP':
            return self._F(t, x)
        else:
            return x.transpose()@self._F@x
    
    def save(self, path):
        """Saving the objective to a given file with dill.
        
        The path can be absolut or relative and 
        the ending of the file is arbitrary.

        Parameters
        ----------
        path : str
            String defining the path to the desired file. 

        """
        
        with open(path, "wb") as output_file:
            dill.dump(self, output_file, -1)
    
    @classmethod
    def load(cls, path):
        """Loads a nMPyC objective object from a given path."""
        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load model from file. File not readable!')
            
        if not isinstance(e, objective):
            raise Exception(
                'Can not load model from file. File does not cotain a model!')
            
        return e

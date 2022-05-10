#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl

import nmpyc as mpc
from nmpyc.utilis import mpc_convert

import numpy as np

from inspect import signature
import dill

class objective:
    """
    A class used to define the objective of the optimnal control problem.
    
    The objective depends on stage costs and optional terminal cost.
    
    Parameters
    ----------
    stagecost : callable
        A function defining the stage costs of the optimal control problem. 
        Has to be of the form :math:`\ell(t,x,u)` or :math:`\ell(x,u)` in the :py:attr:`~autonomous` case.
    terminalcost : callable, optional
        A function defining the terminal cost of the optimal control 
        problem. Has to be of the form :math:`F(t,x)` or :math:`F(x)` in the autonomous 
        case. If None, no terminal cost is applied. 
        The default is None.
    
    """
    
    def __init__(self, stagecost, terminalcost=None):
        
        self.stagecost = stagecost
        self.terminalcost = terminalcost
        self._type = 'NLP'
        self._discont = 1.
        
    @property 
    def stagecost(self):
        """callable : Function defining the stage costs :math:`l(t,x,u)`."""
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
                    'stage cost must have two or three input arguments - not ' 
                    + str(len(params)))
        elif isinstance(cost, list):
            for i in range(3):
                if not isinstance(cost[i], mpc.array): 
                    raise TypeError(
                        'stage cost must be a list of arrays -  not ' 
                        + str(type(cost[i])))
            self._autonomous = True
            L = cost
        else: 
            raise TypeError(
                'stage cost must be callable or list of arrays - not ' 
                + str(type(cost)))
        self._L = L
        
    @property 
    def terminalcost(self):
        """callable : Function defining the terminal cost :math:`F(t,x)`."""
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
                    'terminal cost must have two or three input arguments - not ' 
                    + str(len(params)))
        elif isinstance(cost,mpc.array):
            F = cost
        else: 
            if cost is not None:
                raise TypeError(
                    'terminal cost must be callable or arrays - not ' 
                    + str(type(cost)))
        self._F = F
        
    @property 
    def autonomous(self):
        """bool : If True, the objective is autonomous. 
        
        That menas that the stage costs and terminal cost 
        of the objective :math:`J(t,x,u,N)` 
        are not explicitly depend on the time variable :math:`t`.
        In this case :math:`J(t,x,u,N)=J(x,u,N)` holds."""

        return self._autonomous
    
    @property 
    def type(self):
        """str : If LQP, the objective is quadratic."""
        return self._type
    
    @property
    def discount(self):
        """float : The discount factor of the objective.
        
        For a discount factor :math:`\delta \in (0,1]` the 
        discounted objectivefunction reads as

        .. math::
           
           J(t,x,u,N) = \sum_{k=0}^{N-1} \delta^k \ell(t,x,u) + F(t,x).

        By default :math:`\delta = 1` holds and in this case the problem 
        is called undiscounted.
        The discont factor for the OCP of the MPC simulation can be set 
        by calling the the :py:meth:`model.mpc` method.
        """
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
        string += 'terminalcost: ' + str(terminal) + '\n'
        
        lqp = False
        if self._type == 'LQP': lqp = True
        string += 'linear: ' + str(lqp) + '\n'
        
        string += 'discountfactor: ' + str(self.discount)
        
        return string
        
    @classmethod
    def LQP(cls, Q, R, N=None, P=None):
        """Initialize a quadratic objective.

        In this case the stage costs of the objective have the form

        .. math::
           \ell(x,u) = x^T Q x + u^T R u + 2 x^T Q u

        and the optional terminal cost are defined as 

        .. math::
           F(x,u) = x^T P x.

        In this case the objective is always :py:attr:`~autonomouse`.

        Parameters
        ----------
        Q : array
            Matrix defining the costs of the state of the form :math:`x^TQx`.
        R : array
            Matrix defining the cost of the control of the form :math:`u^TRu`.
        N : array, optional
            Possible Matrix defining the mixed costterm of the form :math:`2x^TNu`. 
            The default is None.
        P : array, optional
            Posible Matrix defining the terminal cost of the form :math:`x^TPx`. 
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
        """Add terminal cost to the objective.
        
        The terminal cost must be a callable function of the form :math:`F(t,x)` 
        or :math:`F(x)` in the autonomous case. If terminal cost already exists they 
        will be overwritten by the new one given.

        Parameters
        ----------
        terminalcost : callable
            A function defining the terminal cost of the optimal control 
            problem. Has to be of the form :math:`F(t,x)` or :math:`F(x)` in the autonomous 
            case.

        """
        
        self.terminalcost = terminalcost
        
    def J(self, t, x, u, N):
        """Evaluate objectivefunction of the OCP.

        The objectivefunction is assembled from the stage costs :math:`\ell(t,x,u)` 
        and optional terminal cost :math:`F(t,x)` and has the form 

        .. math::

           J(t,x,u,N) = \sum_{k=0}^{N-1} \ell(t(k),x(k),u(k)) + F(t(N),x(N)).

        Parameters
        ----------
        t : array
            Timesequence at which the stage costs and terminal cost are evaluated.
        x : array
            State trajectory at which the stage costs and terminal cost are 
            evaluated.
        u : array
            Control sequence at which the stage costs are evaluated.
        N : int
            Maximum index up to which the stage costs are to be summed up. 
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
        """Evaluate stage cost of the objective.

        Parameters
        ----------
        t : float
            Time at which the stage cost should be evaluated.
        x : array
            Current state at which the stage cost should be evaluated.
        u : array
            Current control at which the stage cost should be evaluated.

        Returns
        -------
        array
            Stage cost evaluated at the given values.
            
        """
            
        if self._type == 'NLP':
            return self._L(t,x,u)
        else:
            return (x.transpose()@self._L[0]@x 
                    + u.transpose()@self._L[1]@u 
                    + x.transpose()@self._L[2]@u*2)

    
    @mpc_convert
    def endcosts(self, t, x):
        """Evaluate termninal cost of the objective.

        Parameters
        ----------
        t : float
            Time at which the terminal cost should be evaluated.
        x : array
            Current state at which the terminal cost should be evaluated.

        Returns
        -------
        array
            Terminal cost evaluated at the given values.

        """
        
        if self._F is None:
            return mpc.zeros(1)
        if self._type == 'NLP':
            return self._F(t, x)
        else:
            return x.transpose()@self._F@x
    
    def save(self, path):
        """Saving the objective to a given file with `dill <https://dill.readthedocs.io/en/latest/dill.html>`_.
        
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
                'Can not load objective from file. File not readable!')
            
        if not isinstance(e, objective):
            raise Exception(
                'Can not load objective from file. File does not cotain a objective!')
            
        return e

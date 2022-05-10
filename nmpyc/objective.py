#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl

import nmpyc as mpc
from nmpyc.utils import mpc_convert

import numpy as np

from inspect import signature
import dill

class objective:
    """
    A class used to define the objective of the optimal control problem.
    
    The objective depends on the stage cost and optional on terminal cost and has the form

    .. math::

       J(t,x,u,N) := \sum_{k=0}^{N-1} \ell(t_k,x(t_k),u(t_k)) + F(t_N,x(t_N)).

    The values of the times :math:`t_k` are defined by initializing the :py:class:`nmpyc.system.system`.
    For the slightly different form of the objective in the discounted case see :py:attr:`~discount`.
    
    Parameters
    ----------
    stagecost : callable
        A function defining the stage cost of the optimal control problem. 
        Has to be of the form :math:`\ell(t,x,u)` or :math:`\ell(x,u)` in the :py:attr:`~autonomous` case.
        See also :py:attr:`stagecost`.
    terminalcost : callable, optional
        A function defining the terminal cost of the optimal control 
        problem. Has to be of the form :math:`F(t,x)` or :math:`F(x)` in the autonomous 
        case. If None, no terminal cost is added. 
        The default is None. See also :py:attr:`terminalcost`.
    
    """
    
    def __init__(self, stagecost, terminalcost=None):
        
        self.stagecost = stagecost
        self.terminalcost = terminalcost
        self._type = 'NLP'
        self._discont = 1.
        
    @property 
    def stagecost(self):
        """callable or list of array: Stage cost :math:`\ell(t,x,u)`.
        
        The return value of this attribute depends on how the 
        objective is initialized. 
        If it is initialized as a quadratic objective by :py:meth:`~LQP` a list 
        containing the arrays defining the stage cost are returned. 
        If the obejctive is initalized by possibly nonlinear callable functions 
        the function defining the stage cost is returned. 
        Note, that even if :py:attr:`~autonomous` is True the returned function depends 
        on the time :math:`t` and always has the form :math:`\ell(t,x,u)`.
        """
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
        """callable or array: Terminal cost :math:`F(t,x)`.
        
        The return value of this attribute depends on how the 
        objective is initialized. 
        If it is initialized as a quadratic objective by :py:meth:`~LQP` the 
        array defining the terminal cost is returned. 
        If the obejctive is initalized by possibly nonlinear callable functions 
        the function defining the terminal cost is returned. 
        Note, that even if :py:attr:`~autonomous` is True the returned function depends 
        on the time :math:`t` and always has the form :math:`\ell(t,x,u)`.
        """
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
        
        The stage cost and terminal cost 
        of the objective :math:`J(t,x,u,N)` 
        are not explicitly dependend on the time variable :math:`t`.
        In this case :math:`J(t,x,u,N)=J(x,u,N)` holds."""

        return self._autonomous
    
    @property 
    def type(self):
        """str : Indicating whether the objective is quadratic or nonlinear.
        
        If `LQP`, the objective is quadratic.
        Then the stage cost 
        is given by

        .. math::

           \ell(x,u) = x^TQx + u^TRu + 2x^TNx.

        and the terminal cost is given by

        .. math::

           F(x) = x^TPx.

        It also implies that the system is :py:attr:`~autonomous`.

        If the objective is not initialized as quadratic function with the :py:meth:`~LQP` 
        method this attribute holds the value `NLP`.
        """
        return self._type
    
    @property
    def discount(self):
        """float : The discount factor of the objective.
        
        For a discount factor :math:`\delta \in (0,1]` the 
        discounted objective function is given by

        .. math::
           
           J(t,x,u,N) = \sum_{k=0}^{N-1} \delta^k \ell(t_k,x(t_k),u(t_k)) + F(t_N,x(t_N)).

        By default the discount factor is equal to 1 :math:`\delta = 1`. Then, we name the problem undiscounted.
        The discount factor for the OCP of the MPC simulation can be set 
        when the :py:meth:`nmpyc.model.model.mpc` method is called.
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

        In this case the stage cost of the objective has the form

        .. math::
           \ell(x,u) = x^T Q x + u^T R u + 2 x^T Q u

        and the optional terminal cost is defined as 

        .. math::
           F(x,u) = x^T P x.

        In this case the objective is always :py:attr:`~autonomous`.

        Parameters
        ----------
        Q : array
            Matrix defining the cost of the state of the form :math:`x^TQx`.
        R : array
            Matrix defining the cost of the control of the form :math:`u^TRu`.
        N : array, optional
            Possible Matrix defining the mixed cost term of the form :math:`2x^TNu`. 
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
        will be over written.

        Parameters
        ----------
        terminalcost : callable
            A function defining the terminal cost of the optimal control 
            problem. Has to be of the form :math:`F(t,x)` or :math:`F(x)` in the autonomous 
            case.

        """
        
        self.terminalcost = terminalcost
        
    def J(self, t, x, u, N):
        """Evaluate objective function of the OCP.

        The objective function is assembled from the stage cost :math:`\ell(t,x,u)` 
        and optional terminal cost :math:`F(t,x)` and has the form 

        .. math::

           J(t,x,u,N) = \sum_{k=0}^{N-1} \delta^k \ell(t_k,x(t_k),u(t_k)) + F(t_N,x(t_N)).

        Where :math:`\delta \in (0,1]` is a possible discount factor, see :py:attr:`~discount`.

        Parameters
        ----------
        t : array
            Times instant at which the stage costs and terminal cost are evaluated.
        x : array
            State trajectory at which the stage cost and terminal cost are 
            evaluated.
        u : array
            Control sequence at which the stage cost is evaluated.
        N : int
            Maximum index up to which the stage cost are summed. 
            During the MPC iteration this index is equivalent to 
            the MPC horizon.

        Returns
        -------
        J : array
            Value of the objective function at the given input parameters.

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
            Time instant at which the stage cost is evaluated.
        x : array
            Current state at which the stage cost is evaluated.
        u : array
            Current control at which the stage cost is evaluated.

        Returns
        -------
        array
            Stage cost evaluated at the given input values.
            
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
            Time instant at which the terminal cost is evaluated.
        x : array
            Current state at which the terminal cost is evaluated.

        Returns
        -------
        array
            Terminal cost evaluated at the given input values.

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



        For example

        >>> objective.save('objective.pickle')
        
        will create a file `objective.pickle` containing the nMPyC objective object.
        """

        
        with open(path, "wb") as output_file:
            dill.dump(self, output_file, -1)
    
    @classmethod
    def load(cls, path):
        """Loads a nMPyC objective object from a file.
        
        The specified path must lead to a file that was previously saved with
        :py:meth:`~save`.
        
        Parameters
        ----------
        path : str
            String defining the path to the file containing the nMPyC 
            objective object. 
            
            
        For example

        >>> objective.load('objective.pickle')
        
        will load the objective previously saved with :py:meth:`~save`.
            
        """

        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load objective from file. File not readable!')
            
        if not isinstance(e, objective):
            raise Exception(
                'Can not load objective from file. File does not contain a objective!')
            
        return e

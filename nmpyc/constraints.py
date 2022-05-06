#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 26 12:58:18 2021

@author: Jonas Schiessl
"""

import nmpyc as mpc 
from nmpyc.utilis import mpc_convert

import numpy as np

from inspect import signature
import dill

class constraints:
    """
    A class used to define the constraints of the optimnal control problem.
    
    Support of Nonlinear, linear and boxconstraints are implemented 
    and provided.
    """
    
    def __init__(self):
        
        self._lower_bndx = None
        self._upper_bndx = None
        
        self._lower_bndu = None
        self._upper_bndu = None
        
        self._lower_bndend = None
        self._upper_bndend = None
        
        self._linear_constr = {'eq': [], 'ineq': [], 
                               'terminal_eq': [], 'terminal_ineq': []}
        self._nonlinear_constr = {'eq': [], 'ineq': [], 
                                  'terminal_eq': [], 'terminal_ineq': []}
        
        self._type = 'LQP'
        
    @property
    def lower_bndx(self):
        """array : Lower bound for the state."""
        return self._lower_bndx 
    
    @property
    def upper_bndx(self):
        """array : Upper bound for the state."""
        return self._upper_bndx 
    
    @property
    def lower_bndu(self):
        """array : Lower bound for the control."""
        return self._lower_bndu
    
    @property
    def upper_bndu(self):
        """array : Upper bound for the control."""
        return self._upper_bndu 
    
    @property 
    def lower_bndend(self):
        """array : Lower bound for the terminal state."""
        return self._lower_bndend
    
    @property 
    def upper_bndend(self):
        """array : Upper bound for the terminal state."""
        return self._upper_bndend
    
    @property 
    def linear_constr(self):
        """dict : Collection of all linear constraints."""
        return self._linear_constr
    
    @property 
    def nonlinear_constr(self):
        """dict : Collection of all nonlinear constraints."""
        return self._nonlinear_constr
    
    @property 
    def type(self):
        """str : If LQP, all constraints are linear."""
        return self._type
        
    def __str__(self):
        string = ''
        
        nlbx = 0
        nubx = 0
        nlbe = 0
        nube = 0
        
        if self._lower_bndx is not None:
            for i in range(len(self._lower_bndx)):
                if self._lower_bndx[i] != -mpc.inf:
                    nlbx += 1
        if self._upper_bndx is not None:
            for i in range(len(self._upper_bndx)):
                if self.upper_bndx[i] != -mpc.inf:
                    nubx += 1
        if self._lower_bndend is not None:
            for i in range(len(self._lower_bndend)):
                if self.lower_bndend[i] != -mpc.inf:
                    nlbe += 1
        if self._upper_bndend is not None:
            for i in range(len(self._upper_bndend)):
                if self._upper_bndend[i] != -mpc.inf:
                    nube += 1
        
        string += 'number of lower bounds for state: ' + str(nlbx) +'\n'
        string += 'number of upper bounds for state: ' + str(nubx) +'\n'
        string += ('number of lower bounds for terminal state: ' 
                   + str(nlbe) 
                   +'\n')
        string += ('number of upper bounds for sterminal tate: ' 
                   + str(nube) 
                   +'\n')
        
        nlbu = 0
        nubu = 0
        
        if self._lower_bndu is not None:
            for i in range(len(self._lower_bndu)):
                if self._lower_bndu[i] != -mpc.inf:
                    nlbu += 1
        if self._upper_bndu is not None:
            for i in range(len(self._upper_bndu)):
                if self._upper_bndu[i] != -mpc.inf:
                    nubu += 1
        
        string += 'number of lower bounds for control: ' + str(nlbu) +'\n'
        string += 'number of upper bounds for control: ' + str(nubu) +'\n'
        
        for key in self._linear_constr.keys():
            string += ('number of linear ' 
                       + key 
                       + ' constraints: ' 
                       + str(len(self._linear_constr[key])) 
                       +'\n')
            
        for key in self._nonlinear_constr.keys():
            string += ('number of nonlinear ' 
                       + key 
                       + ' constraints: ' 
                       + str(len(self._nonlinear_constr[key])) 
                       +'\n')
        
        return string[:-1]
        
        
    def add_bound(self, bnd_type, variable, bound):
        """Add bounds as linear constraints to the OCP.

        Parameters
        ----------
        bnd_type : str
            String defining if the bound is a lower or upper bound.
        variable : str
            String defining on which variable the bound should be applied. 
            Posiible values are state, control and terminal.
        bound : array
            Array containing the values of the bound.

        """
        
        if not isinstance(bound, mpc.array):
            raise TypeError(
                'bound must be of type array - not ' + str(type(bound)))
        if bound.symbolic:
            raise ValueError(
                'bound has to be purely numeric,' 
                + ' but also has symbolic entries')
        if not bound.dim[1] == 1:
            if not bound.dim[0] == 1:
                print(bound.dim)
                raise ValueError(
                    'bound must be a flat array'
                    + ' but booth dimnensions are greter than 1')
            else:
                bound = bound.transpose()
            
        if not isinstance(bnd_type, str):
            raise TypeError(
                'bnd_type must be of type string - not ' 
                + str(type(bnd_type)))
        if not bnd_type in ['upper', 'lower']:
            raise ValueError(
                'bnd_type has to be upper or lower - not ' + bnd_type)
            
        if not isinstance(variable, str):
            raise TypeError(
                'variable must be of type string - not ' 
                + str(type(variable)))
        if not variable in ['state', 'control','terminal']:
            raise ValueError(
                'variable has to be state, control or terminal - not ' 
                + variable)
            
        if variable == 'state':
            if bnd_type == 'lower':
                self._lower_bndx = bound
            elif bnd_type == 'upper':
                self._upper_bndx = bound
        elif variable == 'control':
            if bnd_type == 'lower':
                self._lower_bndu = bound
            elif bnd_type == 'upper':
                self._upper_bndu = bound
        elif variable == 'terminal':
            if bnd_type == 'lower':
                self._lower_bndend = bound
            elif bnd_type == 'upper':
                self._upper_bndend = bound
                
    def add_constr(self, cons_type, *args):
        """Add linear or nonlinear constraints to the OCP.
        
        Nonlinear inequality constraints are of the form   
        g(t,x,u) >= 0 or g(x,u) >= 0.
        Nonlinear equality constraints are of the form
        h(t,x,u) = 0 or h(x,u) = 0.
        Linear inequality constraints are of the form 
        Ex + Fu >= h.
        Linear equality constraints are of the form 
        Ex + Fu = h.
        For the form of terminal constrains see add_terminalconstr().

        Parameters
        ----------
        cons_type : str
            String that defines the type of the constraints. 
            Possible values are eq, ineq, terminal_eq and terminal_ineq.
        *args : callable or arrays
            Function defining the (nonlinear) constraints or 
            arrays defining the linear constraints. 
            In the letter case the order of arguments are E, F, h and 
            if h is undefined this array is set to zero.

        """
        
        if cons_type  == 'terminal_eq':
            self.add_terminalconstr('eq', *args)
            return None
        elif cons_type  == 'terminal_ineq':
            self.add_terminalconstr('ineq', *args)
            return None
        
        if not isinstance(cons_type, str):
            raise TypeError()
        if cons_type not in ['eq', 'ineq']:
            raise ValueError()
        
        if len(args) == 1:
            linear = False
            if callable(args[0]):
                sig = signature(args[0])
                params = sig.parameters
                if len(params) == 2:
                    cons = lambda t,x,u : args[0](x, u)
                elif len(params) == 3:
                    cons = args[0]            
                else:
                    raise ValueError(
                        'callable non-terminal constraints must have' 
                        + ' two or three input arguments - not ' 
                        + str(len(params)))
            else:
                raise TypeError(
                    'constraints must be defined by a callable function ' 
                    + 'or as linear constraints by two arrays - not by ' 
                    + str(type(args[0])))
        elif len(args) == 2:
            linear = True
            for i in range(len(args)):
                if not isinstance(args[i], mpc.array): 
                    raise TypeError(
                        'linear constraints must be defined by arrays -  not ' 
                        + str(type(args[i])))
            h = np.zeros(args[0].dim[0])
            args = (args[0], args[1], h)
        elif len(args) == 3:
            linear = True
            for i in range(len(args)):
                if not isinstance(args[i], mpc.array): 
                    raise TypeError(
                        'linear constraints must be defined by arrays -  not ' 
                        + str(type(args[i])))
        else:
            raise ValueError(
                'given constraint must be a callable or two arrays')
        
        if linear:
            self._linear_constr[cons_type] += [args]
        else:
            self._nonlinear_constr[cons_type] += [mpc_convert(cons)]
            self._type = 'NLP'
            
    def add_terminalconstr(self, cons_type, *args):
        """Add linear or nonlinear terminal constraints to the OCP.
        
        Nonlinear terminal inequality constraints are of the form   
        g(t,x) >= 0 or g(x) >= 0.
        Nonlinear terminal equality constraints are of the form
        h(t,x) = 0 or h(x) = 0.
        Linear terminal inequality constraints are of the form 
        Ex >= h.
        Linear terminal equality constraints are of the form 
        Ex = h.

        Parameters
        ----------
        cons_type : str
            String that defines the type of the terminal constraints. 
            Possible values are eq or ineq.
        *args : callable or arrays
            Function defining the (nonlinear) terminal constraints or 
            arrays defining the linear constraints. 
            In the letter case the order of arguments are E, h and 
            if h is undefined this array is set to zero.

        """
        
        if not isinstance(cons_type, str):
            raise TypeError()
        if cons_type not in ['eq', 'ineq']: 
            raise ValueError()   
        if len(args) > 2:
            print(args)
            raise TypeError(
                'terminal constraints must be defined by a callable function ' 
                + 'or as linear constraints by an arrays')
        
        if len(args) == 1:
            if callable(args[0]):
                linear = False
                sig = signature(args[0])
                params = sig.parameters
                if len(params) == 1:
                    cons = lambda t,x : args[0](x)
                elif len(params) == 2:
                    cons = args[0]            
                else:
                    raise ValueError(
                        'callable terminal constraints must have' 
                        + ' one or two input arguments - not ' 
                        + str(len(params)))
            elif isinstance(args[0], mpc.array):
                linear = True
                h = np.zeros(args[0].dim[0])
                args = (args[0], h)
            else:
                raise ValueError(
                    'given constraint must be a callable or arrays')
        elif len(args) == 2:
            linear = True
            for i in range(len(args)):
                if not isinstance(args[i], mpc.array): 
                    raise TypeError(
                        'linear constraints must be defined by arrays -  not ' 
                        + str(type(args[i])))
        else:
            raise ValueError('given constraint must be a callable or arrays')
        
        cons_type = 'terminal_' + cons_type
        if linear:
            self._linear_constr[cons_type] += [args]
        else:
            self._nonlinear_constr[cons_type] += [mpc_convert(cons)]
            self._type = 'NLP'
                
    def save(self, path):
        """Saving the constraints to a given file with dill.
        
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
        """Loads a nMPyC constraint object from a given path."""
        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load model from file. File not readable!')
            
        if not isinstance(e, constraints):
            raise Exception(
                'Can not load model from file. File does not cotain a model!')
            
        return e
                
            
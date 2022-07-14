#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl

import nmpyc as mpc 
from nmpyc.utils import mpc_convert

import numpy as np

from inspect import signature
import dill

class constraints:
    """
    Class used to define the constraints of the optimnal control problem.
    
    Support for nonlinear, linear and box constraints are implemented 
    and provided.

    To define the constraints, first, an empty object have to be created. 
    Then the individual constraints can be added with the help of the methods
    :py:meth:`add_bound` and :py:meth:`add_constr`.
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
        """array : Lower bound :math:`l_x \in \mathbb{R}^{nx}` of the state.
        
        For all states :math:`x(t_k)` the inequalities 

        .. math::

           x_i(t_k) \geq l_{x_i} \quad \\text{for } i = 1,\ldots,nx

        hold as a constraint.
        """
        return self._lower_bndx 
    
    @property
    def upper_bndx(self):
        """array : Upper bound :math:`u_x \in \mathbb{R}^{nx}` of the state.
        
        For all states :math:`x(t_k)` the inequalities 

        .. math::

           x_i(t_k) \leq u_{x_i} \quad \\text{for } i = 1,\ldots,nx

        hold as a constraint.
        """
        return self._upper_bndx 
    
    @property
    def lower_bndu(self):
        """array : Lower bound :math:`l_u \in \mathbb{R}^{nu}` for the control.
        
        For all controls :math:`u(t_k)` the inequalities 

        .. math::

           u_i(t_k) \geq l_{u_i} \quad \\text{for } i = 1,\ldots,nu

        hold as a constraint.
        """
        return self._lower_bndu
    
    @property
    def upper_bndu(self):
        """array : Upper bound :math:`u_u \in \mathbb{R}^{nu}` of the control.
        
        For all controls :math:`u(t_k)` the inequalities

        .. math::

           u_i(t_k) \leq u_{u_i} \quad \\text{for } i = 1,\ldots,nu

        hold as a constraint.
        """
        return self._upper_bndu 
    
    @property 
    def lower_bndend(self):
        """array : Lower bound :math:`l_x \in \mathbb{R}^{nx}` of the terminal state.
        
        For the terminal state :math:`x(t_N)` the inequality 

        .. math::

           x_i(t_N) \geq l_{x_i} \quad \\text{for } i = 1,\ldots,nx

        holds as a constraint.
        """
        return self._lower_bndend
    
    @property 
    def upper_bndend(self):
        """array : Upper bound :math:`u_x \in \mathbb{R}^{nx}` of the terminal state.
        
        For the terminal state :math:`x(t_N)` the inequalities

        .. math::

           x_i(t_N) \leq u_{x_i} \quad \\text{for } i = 1,\ldots,nx

        hold as a constraint.
        """
        return self._upper_bndend
    
    @property 
    def linear_constr(self):
        """dict : Collection of all linear constraints.
        
        This dictionary has the following form:

        >>> linear_constr = {'eq': [..], 'ineq': [..], 
        >>>                  'terminal_eq': [..], 'terminal_ineq': [..]}

        The arrays that define the constraints are saved as lists which are contained in the dictionary.
        For example

        >>> linear_constr['eq'][0]

        returns a list with the arrays :math:`H`, :math:`F` and :math:`h` defining the 
        first equality constraint

        .. math::

           Hx + Fu = h.
        """
        return self._linear_constr
    
    @property 
    def nonlinear_constr(self):
        """dict : Collection of all nonlinear constraints.
        
        This dictionary has the following form:

        >>> nonlinear_constr = {'eq': [..], 'ineq': [..], 
        >>>                     'terminal_eq': [..], 'terminal_ineq': [..]}

        In the lists contained in the dictionary the functions defining the 
        constraints are saved. 
        For example

        >>> nonlinear_constr['eq'][0]

        returns the function :math:`h(t,x,u)` defining the 
        first equality constraint

        .. math::

           h(t,x,u) = 0.
        """
        return self._nonlinear_constr
    
    @property 
    def type(self):
        """str : Indicating whether all constraints are linear.
        
        If `LQP`, all constraints are linear.
        Then all constraints are of the form

        .. math::

           Ex + Fu \leq h

        If at least one constraint is initialized as a nonlinear constraint 
        this attribute has the value `NLP`.
        """

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

        Note while adding the bound it is not 
        checked if the bounds have the correct shape.
        This will be verified later during the optimization 
        progress.

        Parameters
        ----------
        bnd_type : str
            String defining whether the bound is a lower or upper bound.
        variable : str
            String defining on which variable the bound should be applied. 
            Possible values are *state*, *control* and *terminal*.
        bound : array
            Array containing the values of the bound.

        
        For example 

        >>> constraints.add_bound('lower', 'state', lbx)

        will add `lbx` as :py:attr:`~lower_bndx` while

        >>> constraints.add_bound('upper', 'terminal', ub_end)

        will add `ub_end` as :py:attr:`~upper_bndend`.

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

        .. math::

           g(t,x,u) \geq 0 \quad \\text{or} \quad g(x,u) \geq 0.

        Nonlinear equality constraints are of the form

        .. math::

           h(t,x,u) = 0 \quad \\text{or} \quad h(x,u) = 0.

        Linear inequality constraints are of the form 

        .. math::

           Ex + Fu \geq h.

        Linear equality constraints are of the form 

        .. math::

           Ex + Fu = h.

        For the form of terminal constrains see :py:meth:`~add_terminalconstr`.

        Parameters
        ----------
        cons_type : str
            String that defines the type of the constraints. 
            Possible values are *eq*, *ineq*, *terminal_eq* and *terminal_ineq*.
        *args : callable or arrays
            Function defining the (nonlinear) constraints or 
            arrays defining the linear constraints. 
            In the letter case the order of arguments are E, F, h and 
            if h is undefined this array is set to zero.



        For example 

        >>> constraints.add_constr('ineq', E, F, h)

        will add a linear inequality constraint to :py:attr:`linear_constr` while

        >>> constraints.add_constr('terminal_eq',h_end)

        will add a nonlinear equality terminal constraint to :py:attr:`nonlinear_constr`.

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

        .. math::

           g(t,x) \geq 0 \quad \\text{or} \quad g(x) \geq 0.

        Nonlinear terminal equality constraints are of the form

        .. math::

           h(t,x) = 0 \quad \\text{or} \quad h(x) = 0.

        Linear terminal inequality constraints are of the form 

        .. math::

           Ex \geq h.

        Linear terminal equality constraints are of the form 

        .. math::
        
           Ex = h.

        Parameters
        ----------
        cons_type : str
            String that defines the type of the terminal constraints. 
            Possible values are *eq* or *ineq*.
        *args : callable or arrays
            Function defining the (nonlinear) terminal constraints or 
            arrays defining the linear constraints. 
            In the letter case the order of arguments are E, h and 
            if h is undefined this array is set to zero.



        For example 

        >>> constraints.add_terminalconstr('ineq', E, F, h)

        will add a linear inequality terminal constraint to :py:attr:`linear_constr` while

        >>> constraints.add_constr('eq',h)

        will add a nonlinear equality terminal constraint to :py:attr:`nonlinear_constr`.
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
        """Saving the constraints to a given file with `dill <https://dill.readthedocs.io/en/latest/dill.html>`_.
        
        The path can be absolut or relative and 
        the ending of the file is arbitrary.

        Parameters
        ----------
        path : str
            String defining the path to the desired file. 



        For example

        >>> constraints.save('constraints.pickle')
        
        will create a file `constraints.pickle` containing the nMPyC constraints object.
        """
        
        with open(path, "wb") as output_file:
            dill.dump(self, output_file, -1)
    
    @classmethod
    def load(cls, path):
        """Loads a nMPyC constraints object from a file.
        
        The specified path must lead to a file that was previously saved with
        :py:meth:`~save`.
        
        Parameters
        ----------
        path : str
            String defining the path to the file containing the nMPyC 
            constraints object. 
            
            
        For example

        >>> constraints.load('constraints.pickle')
        
        will load the constraints previously saved with :py:meth:`~save`.
            
        """
        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load constraints from file. File not readable!')
            
        if not isinstance(e, constraints):
            raise Exception(
                'Can not load constraints from file. File does not cotain constraints!')
            
        return e
                
            

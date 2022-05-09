#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl


import casadi as cas

from scipy.integrate import solve_ivp

from inspect import signature
import dill

import nmpyc as mpc
from nmpyc.utilis import mpc_convert

class system:
    """
    A class used to define the systemdynamics of a optimnal control problem.

    The dynamics can be discrete or continuous and will be discretized
    automatically in the letter case.

    Parameters
    ----------
    f : callable
        Function defining the righthandside of the systemdynamics of 
        the form :math:`f(t,x,u)` or :math:`f(x,u)` in the :py:attr:`~autonomous` case.
    nx : int
        Dimension of the state.
    nu : int
        Dimension of the control.
    system_type : str, optional
        String defining if the given systemdynamics are 
        discrete or continuous. The default is 'discrete'.
    sampling_rate : float, optional
        Sampling rate defining at which timeinstances the 
        dynamics are evaluated. The default is 1.
    t0 : float, optional
        Initial time for the optimal control problem. The default is 0.
    method : str, optional
        String defining which integration methode should be used to discretize 
        the systemdynamics. The default is 'cvodes'.


    The following integrators are availible:   

    - from `CasADi <http://casadi.sourceforge.net/api/html/db/d3d/classcasadi_1_1Integrator.html>`_: `cvodes`, `idas`, `collocation`, `oldcollocation` and `rk`   

    - from `SciPy <https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html>`_: `RK45`, `RK23`, `DOP853`, `Radau`, `BDF` and `LSODA` 
      
    - from nMPyC: `rk4`, `euler` und `heun` (fixed step integration methods)
    
    """
    
    def __init__(self, f, nx, nu, system_type='discrete', 
                 sampling_rate=1., t0=0., method='cvodes'):
        
        self._options = {}
        self._type = 'NLP'
        
        self.f = f
        self.system_type = system_type
        self.h = sampling_rate
        self.t0 = t0
        self.nx = nx
        self.nu = nu
        self.method = method
            
        if self._type == 'LQP' and self._system_type == 'continuous':
            if self._method in ['rk4', 'euler', 'heun']:
                self._discretizeLQP()
            else:
                A = self._f[0]
                B = self._f[1]
                self._f = lambda x,u : A@x + B@u
                self._type = 'NLP'
                
    @property 
    def f(self):
        """callable : Righthandside :math:`f(x,u)` of the systemdynamics."""
        return self._f
    
    @f.setter 
    def f(self, ff):
        if callable(ff):
            sig = signature(ff)
            params = sig.parameters
            if len(params) == 2:
                self._autonomous = True
                F = lambda t,x,u : ff(x, u)
            elif len(params) == 3:
                F = ff
                self._autonomous = False
            else:
                raise ValueError(
                    'f must have two or three input arguments - not ' 
                    + str(len(params)))
        elif isinstance(ff, list):
            for i in range(2):
                if not isinstance(ff[i], mpc.array): 
                    raise TypeError(
                        'f must be a list of array -  not ' 
                        + str(type(ff[i])))
            self._autonomous = True
            self._type = 'LQP'
            F = ff
        else:
            raise TypeError(
                'f must be callable or list of arrays - not ' + str(type(ff)))
        self._f = F
        
    @property 
    def system_type(self):
        """str : String defining if the dynamics are discrete or continuous."""
        return self._system_type
    
    @system_type.setter 
    def system_type(self, syst_type):
        if not isinstance(syst_type, str): 
            raise TypeError(
                'system type must be a string - not ' + str(type(syst_type)))
        if syst_type not in ['discrete', 'continuous']:
            raise ValueError(
                'system_type must be discrete or continuous - not ' 
                + syst_type)
        self._system_type = syst_type
        
    @property 
    def h(self):
        """float : Samplingrate of the system."""
        return self._h
    
    @h.setter 
    def h(self, sampling_rate):
        if not isinstance(sampling_rate, (float, int)):
            raise TypeError(
                'sampling_rate must be of type integer or float - not ' 
                + str(type(sampling_rate)))
        if sampling_rate > 0:
            self._h = sampling_rate
        else: 
            raise ValueError(
                'sampling rate must be greter than zero - not ' 
                + str(sampling_rate))
        
    @property 
    def t0(self):
        """float : Initial time of the optimal control problem."""
        return self._t0
    
    @t0.setter 
    def t0(self, t_0):
        if not isinstance(t_0, (float, int)):
            raise TypeError(
                't0 must be of type integer or float - not ' + str(type(t_0)))
        self._t0 = t_0
        
    @property 
    def nx(self):
        """int : Dimension of the state."""
        return self._nx
    
    @nx.setter 
    def nx(self, n_x):
        if not isinstance(n_x, int):
            raise TypeError(
                'nx must be of type integer - not ' + str(type(n_x)))
        if n_x > 0:
            self._nx = n_x
        else: 
            raise ValueError('nx must be grater than zero - not ' + str(n_x))
        
    @property 
    def nu(self):
        """int : Dimension of the control."""
        return self._nu
    
    @nu.setter 
    def nu(self, n_u):
        if not isinstance(n_u, int):
            raise TypeError(
                'nu must be of type integer - not ' + str(type(n_u)))
        if n_u > 0:
            self._nu = n_u
        else: 
            raise ValueError('nu must be grater than zero - not ' + str(n_u))
        
    @property 
    def method(self):
        """str : Integration methode for discretization of the dynamics."""
        return self._method
    
    @method.setter
    def method(self, meth):
        if not isinstance(meth, str): 
            raise TypeError(
                'method must be of type string - not ' + str(type(meth)))
        self._method = meth
        if self._method in ['cvodes', 'idas', 
                            'collocation', 'oldcollocation', 'rk']:
            self._integrator = 'casadi'
        elif self._method in ['RK45', 'RK23', 'DOP853', 
                              'Radau', 'BDF', 'LSODA']:
            self._integrator = 'scipy'
        elif self._method in ['rk4', 'euler', 'heun']:
            self._integrator = 'mpc'
            self._options['number_of_finit_elemets'] = 1
        else:
            raise ValueError(
                'allowed methods are cvodes, idas, collocation, ' 
                + 'oldcollocation, rk, RK45, RK23, DOP853, Radau, BDF, '
                + 'LSODA, euler, heun and rk4 - not ' + meth)
    
    @property 
    def autonomous(self):
        """bool : If true, the objective is time independend."""
        return self._autonomous
    
    @property
    def type(self):
        """str : If LQP, the systemdynamics are linear."""
        return self._type
    
    def __str__(self):
        string = ''
        
        string += 'autonomous: ' + str(self._autonomous) + '\n'
        string += 'system type: ' + str(self._system_type) + '\n'
        
        lqp = False
        if self.type == 'LQP': lqp = True
        string += 'linear: ' + str(lqp) + '\n'
        string += 'sampling rate: ' + str(self._h)
        
        if self.system_type == 'continuous': 
            string += '\n'
            string += 'integrator: ' + str(self.method)
        
        return string 
    
    @classmethod
    def LQP(cls, A, B , nx, nu, system_type='discrete', 
            sampling_rate=1., t0=0., method='euler'):
        """Initialize a system with linear dynamics.
        
        In this case the linear righthandside of the dynamics has the 
        form :
        
        .. math::
            
           f(x,u) = Ax+Bu 
            
        which is always :py:attr:`~autonomouse`.   
        If not a fixed step method is choosen for integration the optimizer 
        can not use the linear structure of the problem during the 
        optimization progress.
        
        Parameters
        ----------
        A : array
            Matrix definig the linear state inmpact on the righthandside 
            of the dynamics.
        B : array
            Matrix definig the linear state inmpact on the righthandside 
            of the dynamics..
        nx : int
            Dimension of the state.
        nu : int
            Dimension of the control.
        system_type : str, optional
            String defining if the given systemdynamics are 
            discrete or continuous. The default is 'discrete'.
        sampling_rate : float, optional
            Sampling rate defining at which timeinstances the 
            dynamics are evaluated. The default is 1.
        t0 : float, optional
            Initial time for the optimal control problem. The default is 0.
        method : str, optional
            String defining which methode should be used to discretize 
            the systemdynamics. The default is 'euler'.

        Returns
        -------
        lqp : system
            nMPyC-system class object suitable to define a linear 
            quadratic optimal control problem..

        """
        
        if not isinstance(A, mpc.array):
            raise TypeError('A must be of type array - not ' + str(type(A)))
        if not A.dim == (nx,nx):
            raise ValueError(
                'A has the wrong dimension - ' 
                + str(A.dim) + '!=' + str((nx,nx)))
            
        if not isinstance(B, mpc.array):
            raise TypeError('B must be of type array - not ' + str(type(B)))
        if not B.dim == (nx,nu):
            raise ValueError(
                'B has the wrong dimension - ' 
                + str(B.dim) + '!=' + str((nx,nu)))
        
        if A.symbolic:
            raise ValueError(
                'A has to be purely numeric, but also has symbolic entries')   
        if B.symbolic:
            raise ValueError(
                'B has to be purely numeric, but also has symbolic entries')
        
        lqp = cls([A,B], nx, nu, system_type, sampling_rate, t0, method)
        
        return lqp
    
    def set_integratorOptions(self, options):
        """Set options for the integration method.

        Parameters
        ----------
        options : dict
            Dictionary containing the keywords of the required options 
            and their values.


        The availible options are depending on the choosen :py:attr:`~method` for integration.
        For the nMPyC integrators the only availible option is `number_of_finit_elements` 
        which must be an int greater than zero and defines how many discretation steps are 
        computet during one sampling period defined by the sampling rate.
        The availible options for the CasADi integrators can be found at `Sourceforge <http://casadi.sourceforge.net/api/html/db/d3d/classcasadi_1_1Integrator.html>`_ and for the SciPy integrators at the `Scipy documentation <https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html>`_.

        """
        
        if not isinstance(options, dict):
            raise TypeError(
                'options must be of type dictionary - not ' 
                + str(type(options)))
        self._options = options
    
    @mpc_convert
    def system_discrete(self, t, x, u):
        """Discretized version of the righthandside of the system dynamics.

        Parameters
        ----------
        t : float
            Time at which the systemdynamic shoul be evaluated.
        x : array
            State value at which the systemdynamic shoul be evaluated.
        u : array
            Control value at which the systemdynamic shoul be evaluated.

        Returns
        -------
        array
            Value of the discretized righthandside of the dynamics 
            evaluated at the given inputs.

        """
        
        if self._system_type == 'discrete':
            return self.system(t, x, u)
        
        else:
            if self._type == 'LQP':
                return self._f_linear(x, u)
            
            if self._integrator == 'casadi':
                return self._solve_ode_casadi(t, x, u)
            
            if self._integrator == 'scipy':
                return self._solve_ode_scipy(t, x, u)
            
            if self._integrator == 'mpc':
                if self._method == 'rk4':
                    return self._solve_ode_rk4(t, x, u)
                elif self._method == 'euler':
                    return self._solve_ode_euler(t, x, u)
                elif self._method == 'heun':
                    return self._solve_ode_heun(t, x, u)
    
    @mpc_convert
    def _f_linear(self, x, u):
        y = self._f[0]@x + self._f[1]@u
        return y
    
    @mpc_convert
    def system(self, t, x, u):
        """Righthandside :math:`f(x,u)` of the dynamics.

        Parameters
        ----------
        t : float
            Time at which the systemdynamic shoul be evaluated.
        x : array
            State value at which the systemdynamic shoul be evaluated.
        u : array
            Control value at which the systemdynamic shoul be evaluated.

        Returns
        -------
       array
            Value of the posiible not discrete righthandside of the dynamics 
            evaluated at the given inputs.

        """
        
        if self.type == 'NLP':
            return self._f(t,x,u)
        else:
            return self._f[0]@x + self._f[1]@u
        
    def _solve_ode_casadi(self, t, x, u):
        
        # Define symbolic variables for writing down the differential equation
        xx = cas.MX.sym('x', self._nx)    
        uu = cas.MX.sym('u', self._nu)   

        # Declare the ODE in casadi style
        ode = {}             
        ode['x'] = xx       
        ode['p'] = uu       
        
        tt = cas.MX.sym('t')
        xdot = self.system(tt, xx, uu)._A
        ode['t'] = tt
        
        ode['ode'] = xdot
        
        # Set the integrator options
        integr_opts = {}
        integr_opts.update(self._options)
        integr_opts['t0'] = t
        integr_opts['tf'] = t + self._h

        # Construct a Function to solve the ODE over a timestep dt
        F = cas.integrator('F', self._method, ode, integr_opts)

        # Solve the ODE
        x0 = mpc.convert(x)
        u = mpc.convert(u)
        res = F(x0=x0, p=u)
        y = mpc.array(res["xf"])
        
        return y
    
    def _solve_ode_scipy(self, t, x, u):
        
        def f(t,z): return mpc.convert(self.system(t,z,u), 'numpy').flatten() 
        
        z0 = mpc.convert(x,'numpy').flatten()
        sol = solve_ivp(f, [t,t+self._h], z0, method=self._method)
        y = mpc.array(sol.y[:,-1])
        
        return y
        
    def _solve_ode_rk4(self, t, x, u):
        
        steps = self._options['number_of_finit_elemets']   # number of steps 
        tk = t
        dt = self._h/steps               # time step = sampling rate/steps
        x_next = mpc.array(x)           # copy x
        # Runge-Kutta 4 integration
        for i in range(steps):
            k1 = self.system(tk, x_next, u)
            k2 = self.system(tk + dt/2, x_next + k1*(dt/2), u)
            k3 = self.system(tk + dt/2, x_next + k2*(dt/2), u)
            k4 = self.system(tk + dt, x_next + k3*dt, u)
                
            x_next = x_next + (k1 + k2*2 + k3*2 + k4)*(dt/6)
            tk += dt
            
        return x_next
    
    def _solve_ode_euler(self, t, x, u):
        
        steps = self._options['number_of_finit_elemets']
        tk = t
        dt = self.h/steps               
        x_next = mpc.array(x)           
        for i in range(steps):
            x_next = x_next + self.system(tk, x_next, u) * dt
            tk += dt
            
        return x_next
    
    def _solve_ode_heun(self, t, x, u):
        
        steps = self._options['number_of_finit_elemets']   
        tk = t
        dt = self._h/steps               
        x_next = mpc.array(x)           
        for i in range(steps):
            xx = x_next + self.system(tk, x_next, u) * dt
            x_next = (x_next + (xx + self.system(tk + dt, xx, u)*dt))*0.5
            tk += dt
            
        return x_next
    
    def _discretizeLQP(self):
        
        if self._method == 'euler':
            self._f[0] = mpc.eye(self._nx) + self._f[0]*self._h 
            self._f[1] = self._f[1]*self._h
            
        if self._method == 'heun':
            A_k1 = self._f[0]
            B_k1 = self._f[1]
            
            A_k2 = self._f[0] + self._f[0]*self._h @ A_k1
            B_k2 = self._f[0]*self._h @ B_k1 + self._f[1]
            
            self._f[0] = mpc.eye(self._nx) + (A_k1 + A_k2)*(self._h/2)
            self._f[1] = (B_k1 + B_k2)*(self._h/2)
            
        elif self.method == 'rk4':
            A_k1 = self._f[0]
            B_k1 = self._f[1]
            
            A_k2 = self._f[0] + self._f[0]@A_k1*(self._h/2)
            B_k2 = self._f[0]@B_k1*(self._h/2) + self._f[1]
            
            A_k3 = self._f[0] + self._f[0] @ A_k2*(self._h/2)
            B_k3 = self._f[0]@B_k2*(self._h/2) + self._f[1]
            
            A_k4 = self._f[0] + self._f[0] @ A_k3 * self._h
            B_k4 = self._f[0]@B_k3*self._h + self._f[1]
            
            self._f[0] = (mpc.eye(self._nx) 
                          + (A_k1 + A_k2*2 + A_k3*2 + A_k4)*(self._h/6))
            self._f[1] = (B_k1 + B_k2*2 + B_k3*2 + B_k4)*(self._h/6)
    
    def save(self, path):
        """Saving the system to a given file with `dill <https://dill.readthedocs.io/en/latest/dill.html>`_.
        
        The path can be absolut or relative and 
        the ending of the file is arbitrary.

        Parameters
        ----------
        path : str
            String defining the path to the desired file. 



        For example

        >>> system.save('system.pickle')
        
        will create a file `system.pickle` contain the nMPyC system object.
        """
        
        with open(path, "wb") as output_file:
            dill.dump(self, output_file, -1)
    
    @classmethod
    def load(cls, path):
        """Loads a nMPyC system object from a given path."""
        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load model from file. File not readable!')
            
        if not isinstance(e, system):
            raise Exception(
                'Can not load model from file. File does not cotain a system!')
            
        return e

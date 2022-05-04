"""
Lumache - Python library for cooks and food lovers.
"""

#__version__ = "0.1.0"

import casadi as cas

from scipy.integrate import solve_ivp

from inspect import signature
import dill

import nMPyC as mpc
from nMPyC.utilis import mpc_convert

class system:
    """
    A class used to define the systemdynamics of a optimnal control problem.
    The dynamics can be discrete or continuous and will be discretized
    automatically in the letter case.
    """
    
    def __init__(self, f, nx, nu, system_type='discrete', 
                 sampling_rate=1., t0=0., method='cvodes'):
        """
        Parameters
        ----------
        f : callable
            Function defining the righthandside of the systemdynamics of 
            the form f(t,x,u) or f(x,u) in the autonomous case.
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
            the systemdynamics. The default is 'cvodes'.

        """
        
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



class InvalidKindError(Exception):
    """Raised if the kind is invalid."""
    pass


def get_random_ingredients(kind=None):
    """
    Return a list of random ingredients as strings.
    :param kind: Optional "kind" of ingredients.
    :type kind: list[str] or None
    :raise lumache.InvalidKindError: If the kind is invalid.
    :return: The ingredients list.
    :rtype: list[str]
    """
    return ["shells", "gorgonzola", "parsley"]

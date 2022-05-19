#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Jonas Schiessl

"""
Module for array definition and computation.

This module provides an array class and associated functions for 
corresponding matrix calculations.

The goal of this class and the individual functions is to enable 
compatibility of calculations with both casadi and numpy objects 
without changing the syntax of the program. 
This enables the user to program as easily as possible and at 
the same time to switch between symbolic and numeric calculation.
"""
import numpy as np
from numpy import linalg as LA
import casadi as cas

# Constants
inf = np.inf
"""float : Constant to define infinity."""
pi = np.pi
"""float : Constant to define the number pi."""

class array:
    """
    Class used to save arrays with symbolic or numeric values. 
    
    The symbolic entries are provided by CasADi and will be 
    transformed automatically to numeric values of numpy type 
    if it is posiible.
    
    Parameters
    ---------
    dim : int, tuple, cas.MX, cas.SX, cas.DM, list or numpy.ndarray, optional
        Dimension of which an empty array is created or object from which 
        the entries and dimension are copied. The default is 0.

    """
    
    def __init__(self, dim=0):
        
        if isinstance(dim, int):
            self._dim = (dim, 1)
        elif isinstance(dim, float):
            if (dim).is_integer():
                self._dim = (int(dim), 1)
            else: 
                raise ValueError('dimensions must be integers')
        elif isinstance(dim, tuple):
            if (not isinstance(dim[0], (int, float)) or not 
                isinstance(dim[1], (int, float))):
                raise ValueError('dimension must be tupel of integers')
            if isinstance(dim[0], float) and not (dim[0]).is_integer():
                raise ValueError('dimension must be tupel of integers')
            if isinstance(dim[1], float) and not (dim[1]).is_integer():
                raise ValueError('dimension must be tupel of integers')
            self._dim = dim
        else:
            self._dim = (0,0)
            
        if self._dim[0]<0 or self._dim[1]<0:
            raise ValueError('negativ dimensions are not allowed')
            
        self._A = np.zeros(self._dim)
        self._casadi = False
        
        if isinstance(dim, (np.ndarray, cas.MX, cas.SX, cas.DM, array, list)):
            self._copy(dim)
        else: 
            if not isinstance(dim, (int, float, tuple)):
                raise TypeError(
                    'input parameter must be of type' + 
                    ' int, tuple, list, numpy.ndarray,' + 
                    ' casadi.MX, casadi.SX or casadi.DX - not ' 
                    + str(type(dim)))
                
        if not len(self._dim) == 2:
            raise ValueError('only two dimesnions are allowed')
            
    @property 
    def A(self):
        """ casadi.MX or numpy.array : Array containing all entries."""
        try:
            AA = np.array(cas.evalf(self._A))
            return AA
        except: 
            return self._A
    
    @property 
    def dim(self):
        """tuple : Dimension of the array."""
        return self._dim
    
    @property
    def symbolic(self):
        """bool : True if array has symbolic entries, False otherwise."""
        return self._casadi
    
    @property
    def T(self):
        """array : Transposed array."""
        return self.transpose()
        
    def __str__(self): 
        return str(self._A)
    
    def __len__(self): 
        if self._casadi:
            return self._A.shape()[1]
        else:
            return len(self._A)
    
    def __neg__(self): 
        C = array(-self._A)
        return C
    
    def __pos__(self):
        C = array(-self._A)
        return C
    
    def __abs__(self): 
        return abs(self)
    
    def __int__(self): 
        C = int(self._A)
        return C
    
    def __float__(self): 
        C = float(self._A)
        return C
    
    def __add__(self, other): 
    
        if not isinstance(other, 
                          (array, np.ndarray, cas.MX, cas.SX, cas.DM, 
                           int, float)):
             raise TypeError(
                 'input parameter must be of type' + 
                 ' array, int, float, numpy.ndarray,' + 
                 ' casadi.MX, casadi.SX or casadi.DX - not ' 
                 + str(type(other)))
        
        C = array(self._dim)
        
        if isinstance(other, array):
            if self._dim != other._dim:
                raise ValueError(
                    'arrays must have the same dimensions. ' 
                    + str(self._dim) + 
                    ' != ' + str(other._dim))
            other = other._A
        
        if isinstance(other, np.ndarray) and len(other.shape) == 1:
            if self._dim[0] == 1:
                other = np.reshape(other,(self._dim[0],other.shape[0]))
            elif self._dim[1] == 1:
                other = np.reshape(other,(other.shape[0],self._dim[1]))
            else: 
                raise ValueError(
                    'arrays must have the same dimensions. ' 
                    + str(self._dim) + ' != ' + str(other.shape))
             
        C._A = self._A + other
        
        try:
            C._A = np.array(cas.evalf(C._A))
        except: 
            C._casadi = True
        
        return C
    
    def __radd__(self, other): 
        return self.__add__(other)
    
    def __sub__(self, other):
        return(self.__add__((-other)))
    
    def __rsub__(self, other): 
        return -(self.__sub__(other))
        
    def __mul__(self,other): 
    
        if not isinstance(other, 
                          (array, np.ndarray, cas.MX, cas.SX, cas.DM, 
                           int, float)):
             raise TypeError(
                 'input parameter must be of type' 
                 + ' array, int, float, numpy.ndarray,' + 
                 ' casadi.MX, casadi.SX or casadi.DX - not ' 
                 + str(type(other)))
        
        C = array(self._dim)
        
        if isinstance(other, np.ndarray) and len(other.shape) == 1:
            if self._dim[0] == 1:
                other = np.reshape(other,(1, other.shape[0]))
            elif self._dim[1] == 1:
                other = np.reshape(other, (other.shape[0], 1))
            else: 
                raise ValueError(
                    'arrays must have the same dimensions. ' 
                     + str(self._dim) + ' != ' + str(other.shape))
    
        C._A = self._A * other
        
        try:
            C._A = np.array(cas.evalf(C._A))
        except: 
            C._casadi = True
                    
        return C
    
    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return self.__mul__(other)
    
    def __matmul__(self, other): 
        
        if isinstance(other, array):
            if self._dim[1] != other._dim[0]:
                raise ValueError(
                    'inner dimensions are note the same - ' 
                    + str(self._dim[1]) + '!=' + str(other._dim[0]))
            dim = (self._dim[0], other._dim[1])
            other = other._A
            
        elif isinstance(other, np.ndarray):
            if len(other.shape) == 1:
                dim = (self._dim[0], 1)
                other = np.reshape(other, (other.shape[0], 1))
            else:
                dim = (self._dim[0], other.shape[1])
            if self._dim[1] != other.shape[0]:
                raise ValueError(
                    'inner dimensions are note the same - ' 
                    + str(self._dim[1]) + '!=' + str(other.shape[0]))
                
        elif isinstance(other, (cas.MX, cas.SX, cas.DM)):
            if self._dim[1] != other.size()[0]:
                raise ValueError(
                    'inner dimensions are note the same - ' 
                    + str(self._dim[1]) + '!=' + str(other.size()[0]))
            dim = (self._dim[0], other.size()[1])
            
        else: 
            raise TypeError(
                'input parameter must be of type ' + 
                'array, numpy.ndarray, ' 
                + 'casadi.MX, casadi.SX or casadi.DX - not ' 
                + str(type(other)))
        
        C = array(dim)
        
        C._A = self._A @ other
        
        try:
            C._A = np.array(cas.evalf(C._A))
        except: 
            C._casadi = True
        
        return C
    
    def __rmatmul__(self, other): 
    
        if isinstance(other, array):
            if self._dim[0] != other._dim[0]:
                raise ValueError(
                    'inner dimensions are note the same - ' 
                    + str(self._dim[0]) + 
                    '!=' + str(other._dim[1]))
            dim = (other._dim[0], self._dim[1])
            other = other._A
            
        elif isinstance(other, np.ndarray):
            if len(other.shape) == 1:
                dim = (1, self._dim[1])
                other = np.reshape(other, (1, other.shape[1]))
            else:
                dim = (other.shape[0], self._dim[1])
            if self._dim[0] != other.shape[1]:
                raise ValueError(
                    'inner dimensions are note the same - ' 
                    + str(self._dim[0]) + '!=' + str(other.shape[1]))
                
        elif isinstance(other, (cas.MX, cas.SX, cas.DM)):
            if self._dim[0] != other.size()[1]:
                raise ValueError(
                    'inner dimensions are note the same - ' 
                    + str(self._dim[0]) + '!=' + str(other.size()[1]))
            dim = (other.size()[0], self._dim[1])
            
        else: 
            raise TypeError(
                'input parameter must be of type' + 
                ' array, numpy.ndarray,' 
                + ' casadi.MX, casadi.SX or casadi.DX - not ' 
                + str(type(other)))
        
        C = array(dim)
        
        C._A = other @ self._A 
        
        try:
            C._A = np.array(cas.evalf(C._A))
        except: 
            C._casadi = True
        
        return C
    
    def __pow__(self, n): 
        
        if not isinstance(n, (int, float)):
            raise TypeError(
                'input parameter must be of type integer or float - not ' 
                + str(type(n)))
            
        return array(self._A**n)
    
    def __getitem__(self, key): 
    
        if isinstance(key, (int, slice)):
            key = (key,0)
        
        try:
            range(self._dim[0])[key[0]]
        except:
            raise IndexError('Index one is out of range: ' 
                             + str(key[0]) + ' not in range [0,' 
                             + str(self._dim[0]-1) + ']')
            
        try:
            range(self._dim[1])[key[1]]
        except:
            raise IndexError('Index one is out of range: ' 
                             + str(key[1]) + ' not in range [0,' 
                             + str(self._dim[1]-1) + ']')
        
        if isinstance(key,tuple):
            if len(key) != 2:
                raise ValueError(
                    'input parameter of type tuple has to be two dimensional')
            if (not isinstance(key[0], (int, slice)) or not 
                isinstance(key[1], (int, slice))):
                raise TypeError(
                    'input parameter must be of type integer or' 
                    + ' tuple of integers or slices')
            
            y = self._A[key[0],key[1]]
            
            if np.isscalar(y):
                return y
            elif isinstance(y, (cas.MX, cas.SX, cas.DM)) and y.is_scalar():
                try:
                    y = float(cas.evalf(y))
                except: None
                return y
            
            return array(y)
        
        else:
            raise TypeError(
                'input parameter must be of type integer or tuple - not ' 
                + str(type(key)))
        
    def __setitem__(self, key, value):
    
        if not isinstance(value, 
                          (array, np.ndarray, cas.MX, cas.SX, cas.DM, 
                           int, float, np.int64)):
             raise TypeError(
                 'input parameter must be of type' 
                 + ' array int, float, numpy.ndarray,' 
                 + ' casadi.MX, casadi.SX or casadi.DX - not ' 
                 + str(type(value)))
        
        if isinstance(key, int):
            key = (key,0)
            
        try:
            range(self._dim[0])[key[0]]
        except:
            raise IndexError('Index one is out of range: ' 
                             + str(key[0]) + ' not in range [0,' 
                             + str(self._dim[0]-1) + ']')
            
        try:
            range(self._dim[1])[key[1]]
        except:
            raise IndexError('Index one is out of range: ' 
                             + str(key[1]) + ' not in range [0,' 
                             + str(self._dim[1]-1) + ']')
        
        if isinstance(value, array):
            value = value._A
            
        if isinstance(key,tuple):
            
            if len(key) != 2:
                raise ValueError(
                    'key of type tuple has to be two dimensional')
            if (not isinstance(key[0], (int, slice)) 
                or not isinstance(key[1], (int, slice))):
                raise TypeError(
                    'key must be of type integer or' 
                    + ' tuple of integers or slices')
                
        else:
            raise TypeError(
                'key must be of type integer or tuple - not ' 
                + str(type(key)))
            
            
        if np.isscalar(value):
           value = float(value)
        elif (isinstance(value, (cas.MX, cas.SX, cas.DM)) and 
              value.is_scalar()):
            try:
                value = float(cas.evalf(value))
                
            except:
                if self._casadi == False:
                    self._A = cas.MX(self._A)
                    self._casadi = True
        else:
             if self._casadi == False:
                 self._A = cas.MX(self._A)
                 self._casadi = True
        
        try:
            self._A[key[0],key[1]] = value
        except: 
            self._A[key[0],key[1]] = value.flatten()
            
        try:
            self._A = np.array(cas.evalf(self._A))
            self._casadi = False
        except:
            self._casadi = True
        
    def _copy(self, other): 
        """Creats a new array object out of a casadi, numpy or array object.

        Parameters
        ----------
        other : array, casadi.MX, casadi.SX, casadi.DM, list or numpy.ndarray
            Object from which the array is created.

        """
        
        if isinstance(other, array):
            if isinstance(other._A, np.ndarray):
                self._A = other._A
            else:
                self._A = other._A
            self._dim = other._dim
            self._casadi = other._casadi
            
        elif isinstance(other, np.ndarray):
            if len(other.shape) == 1:
                self._dim = (other.shape[0], 1)
                self._A = np.reshape(other, self._dim)
            else:
                self._dim = other.shape
                self._A = other
            self._casadi = False
        
        elif isinstance(other, (cas.MX, cas.SX, cas.DM)):
            self._dim = other.size()
            self._A = other
            try:
                self._A = np.array(cas.evalf(self._A))
                self._casadi = False
            except:
                self._casadi = True
                    
        elif isinstance(other, list):
            if len(np.shape(other)) == 1:
                self._dim = (np.shape(other)[0], 1)
                other_ = []
                for i in range(self._dim[0]):
                    other_ += [[other[i]]]
                other = other_
            else:
                self._dim = np.shape(other)
            try:
                self._A = np.array(other)
                self._casadi = False
            except:
                self._A = cas.vertcat(*[cas.horzcat(*row) for row in other])
                self._casadi = True
                    
        else: 
            raise TypeError(
                'input parameter must be of type array, list, numpy.ndarray,' 
                + ' casadi.DM or casadi.SX, casadi.MX - not ' 
                + str(type(other)))
        
    def fill(self, a):
        """Fill all entries with one value.

        Parameters
        ----------
        a : int or float
            Value that all entries should take.

        """
        
        if isinstance(a, (float, int)):
            self._A = np.ones(self._dim)*a
            self._casadi = False
            
        else: 
            raise TypeError(
                'input paramter must be of type integer or float - not ' 
                + str(type(a)))
                    
    def flatten(self):
        """Flat array to one dimension.

        Returns
        -------
        y : array
            Flatten array with dimension (1,n).

        """
        
        y = array(self._A)
        dim2 = y._dim[0]*y._dim[1]
        dim1 = 1
        y._dim = (dim1,dim2)
        if isinstance(y._A, np.ndarray):
            y._A = np.reshape(y._A, y._dim)
        if isinstance(y._A, (cas.MX, cas.SX, cas.DM)):
            y._A = cas.reshape(y._A, y._dim)
            
        return y
    
    def transpose(self):
        """Transpose array.

        Returns
        -------
        y : array
            Transposed array.

        """
        
        y = array(self)
        dim2 = y._dim[0]
        dim1 = y._dim[1]
        y._dim = (dim1,dim2)
        if isinstance(y._A, np.ndarray):
            y._A = y._A.T
        elif isinstance(y._A, (cas.MX, cas.SX, cas.DM)):
            y._A = cas.transpose(y._A)
            
        return y
    
    
def reshape(a, new_size):
    """Reshape an array to a new size.

    Parameters
    ----------
    a : array
        .
    new_size : tuple
        New shape.

    Returns
    -------
    array
        An array instance with the new shape.

    """
    
    if not isinstance(new_size, tuple):
        raise TypeError(
            'new_size must be tuple - not ' 
            + str(type(new_size)))
    if not isinstance(new_size[0], int) or not isinstance(new_size[1], int):
        raise TypeError('new_size must be tuple of inegers')
    if not len(new_size) == 2:
        raise TypeError('new_size must be two dimensional')
        
    if isinstance(a, array):
        a = a._A
    else: 
        raise TypeError(
            'input paramter a must be of type array,- not ' + str(type(a)))
            
    if isinstance(a, np.ndarray):
        A = np.reshape(a,new_size)
    elif isinstance(a, (cas.MX, cas.SX, cas.DM)):
        A = cas.reshape(a, new_size)
    
    return array(A)

def convert(a, dtype='auto'):
    """Convert a numpy-, casadi- or nMPyC-array to another of these intances.

    Parameters
    ----------
    a : array, cas.MX, cas.SX, cas.DM or numpy.ndarray
        Array which should be converted.
    dtype : str, optional
        Name of the class to which the array will be converted. 
        The default is 'auto'.

    Returns
    -------
    numpy.ndarray, cas.MX, cas.SX, cas.DM
        The converted object.

    """
    
    if not isinstance(dtype, str):
        raise TypeError('dtype must be a string - not ' + str(type(dtype)))
    if dtype not in ['auto', 'numpy', 'casadi.DM', 'casadi.SX', 'casadi.MX']:
        raise ValueError('can not convert array to ' + dtype)
    
    if isinstance(a, array):
        a = a._A
        
    if isinstance(a, np.ndarray):
        if dtype == 'numpy':
            return a
        elif dtype == 'casadi.DM':
            return cas.DM(a)
        elif dtype == 'casadi.SX':
            return cas.SX(a)
        elif dtype == 'casadi.MX':
            return cas.MX(a)
        else: return a
    elif isinstance(a, cas.DM):
        if dtype == 'numpy':
            return np.array(a)
        elif dtype == 'casadi.DM':
            return a
        elif dtype == 'casadi.SX':
            return cas.SX(a)
        elif dtype == 'casadi.MX':
            return cas.MX(a)
        else: return a
    elif isinstance(a, cas.SX):
        if dtype == 'numpy':
            return np.array(cas.evalf(a))
        elif dtype == 'casadi.DM':
            return cas.evalf(a)
        elif dtype == 'casadi.SX':
            return a
        elif dtype == 'casadi.MX':
            return cas.MX(a)
        else: return a
    elif isinstance(a, cas.MX):
        if dtype == 'numpy':
            return np.array(cas.evalf(a))
        elif dtype == 'casadi.DM':
            return cas.evalf(a)
        elif dtype == 'casadi.SX':
            return cas.SX(cas.evalf(a))
        elif dtype == 'casadi.MX':
            return a
        else: return a
        
    else:
        raise TypeError(
            'input parameter a must be of type array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(a)))
    
    return a

def concatenate(arrays, axis = 0):
    """Join a sequence of arrays along an existing axis.

    Parameters
    ----------
    arrays : tuple of casadi.MX, casadi.SX, casadi.DM or numpy.ndarrays
        Sequence of arrays which will be concatenated. 
        The arrays must have the same shape, except in the dimension 
        corresponding to axis.
    axis : int, optional
        The axis along which the arrays will be joined. The default is 0.

    Returns
    -------
    array
        The concatenated array.

    """
    
    casadi = False
    
    if not isinstance(arrays, tuple):
        raise TypeError(
            'arrays hmust be of type tuple - not ' + str(type(arrays)))
        
    if not isinstance(axis, int):
        raise TypeError('axis must be an integer - not ' + str(type(axis)))
    
    array_list = []
    for i in range(len(arrays)):
        if isinstance(arrays[i], array):
            array_list += [arrays[i].A]
        else:
            array_list += [arrays[i]]
        if isinstance(array_list[i], (cas.MX, cas.SX, cas.DM)):
            casadi = True
        elif not isinstance(array_list[i], np.ndarray):
            raise ValueError(
                'The elements of the arrays sequence must be of type array,' + 
                ' casadi.MX, casadi.SX, casadi.DM or numpy.ndarray - not ' 
                + str(type(arrays[i])))
            
    arrays = tuple(array_list)
        
    if axis == 0:
        if casadi:
            return cas.vertcat(*arrays)
        else:
            return np.concatenate(arrays, axis=0)
    elif axis == 1:
        if casadi:
            return array(cas.horzcat(*arrays))
        else:
            return array(np.concatenate(arrays, axis=1))
    else:
        raise ValueError(
            'axis must be 0 (first axis) or 1 (second axis) - not ' 
            + str(axis))
    
def eye(dim):
    """Creates an array defining the idendity.

    Parameters
    ----------
    dim : int
        Dimnension of the idendity matrix.

    Raises
    ------
    ValueError
        If the given dimension is not supported.
    TypeError
        If an input parameter has not the right type.

    Returns
    -------
    y : array
        Idendity matrix as an instance of array.

    """
    
    if isinstance(dim, int):
        if dim <= 0:
            raise ValueError('dimension must be graeter than zero')
    else: 
        raise TypeError(
            'dimension must be an integer - not ' 
            + str(type(dim)))
    
    y = array((dim, dim))
    y._A = np.eye(dim)
    return y

def zeros(dim):
    """Creates an array with only zero entries.

    Parameters
    ----------
    dim : int or tuple
        Dimension of the array.

    Raises
    ------
    ValueError
        If the given dimension is not supported.
    TypeError
        If the given dimension has not the right type.

    Returns
    -------
    y : array
        An array of the given dimension with only zero entries.

    """
    
    if isinstance(dim, int): 
        None
    elif isinstance(dim, tuple):
        if len(dim) != 2:
            raise ValueError('dim must be two dimensional')
        if not isinstance(dim[0], int) or not isinstance(dim[0], int):
            raise TypeError('dimension must be tuple of integers')
        if dim[0] <= 0 or dim[1] <= 0:
            raise ValueError('dimension must be graeter than zero')
    else: 
        raise TypeError(
            'dimension must be an integer or tuple - not ' 
            + str(type(dim)))
    
    y = array(dim)
    return y

def ones(dim):
    """Creates an array with only entries equal to one.

    Parameters
    ----------
    dim : int or tuple
        Dimension of the array.

    Raises
    ------
    ValueError
        If the given dimension is not supported.
    TypeError
        If the given dimension has not the right type.

    Returns
    -------
    y : array
        An array of the given dimension with only entries equal to one.

    """

    if isinstance(dim, int): None
    elif isinstance(dim, tuple):
        if len(dim) != 2:
            raise ValueError('dim must be two dimensional')
        if not isinstance(dim[0], int) or not isinstance(dim[0], int):
            raise TypeError('dimension must be tuple of integers')
        if dim[0] <= 0 or dim[1] <= 0:
            raise ValueError('dimension must be graeter than zero')
    else: 
        raise TypeError(
            'dimension must be an integer or tuple - not ' 
            + str(type(dim)))
    
    y = array(dim)
    y.fill(1)
    return y

def diag(x):
    """Creates an diagonal matrix from a given vector.

    Parameters
    ----------
    x : array, numpy.ndarray, cas.MX, cas.SX or cas.DX, list
        Vector containing the diagonal entries of the matrix.

    Returns
    -------
    array
        Diagonal matrix with the desired diagonal elements.

    """
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (np.ndarray)):
        y = np.diag(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM, list)):
        y = cas.diag(x_)
        
    else: 
        raise TypeError(
            'input must be of type array, numpy.ndarray, list,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
        
    return array(y)

def log(x):
    """Calculates the natural logarithm of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int,float,np.ndarray)):
        y = np.log(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.log(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def exp(x):
    """Calculates the exponential of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.exp(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.exp(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def sin(x):
    """Calculates the sinus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.sin(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.sin(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def sinh(x):
    """Calculates the sinus hyperbolicus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.sinh(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.sinh(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def arcsin(x):
    """Calculates the arcussinus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.arcsin(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.arcsin(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def arcsinh(x):
    """Calculates the arcussinus hyperbolicus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.arcsinh(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.arcsinh(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def cos(x):
    """Calculates the cosinus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.cos(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.cos(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def cosh(x):
    """Calculates the cosinus hyperbolicus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.cosh(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.cosh(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def arccos(x):
    """Calculates the arcuscosinus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.arccos(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.arccos(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def arccosh(x):
    """Calculates the arcuscosinus hypernolicus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.arccosh(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.arccosh(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def tan(x):
    """Calculates the tangens of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.tan(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.tan(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def tanh(x):
    """Calculates the tangens hyperblicus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.tan(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.tan(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def arctan(x):
    """Calculates the arcustangens of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.arctan(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.arctan(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def arctanh(x):
    """Calculates the arcustangens hyperbolicus of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.arctanh(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.arctanh(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def sqrt(x):
    """Calculates the square root of a given number or array"""
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.sqrt(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.sqrt(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def power(x,n):
    """Calculates the (elementwise) n-th power of a number or array.

    Parameters
    ----------
    x : int, float, numpy.ndarray, cas.MS, cas.SX or cas.DM
        Number or array of which the n-th power should be computed.
    n : int or float
        Number defining the exponent.

    """
    
    if not isinstance(n, (int, float)):
        raise TypeError(
            'input n must be of type integer or float - not ' 
            + str(type(n)))
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.power(x_,n)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.power(x_,n)
        
    else: 
        raise TypeError(
            'input x must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def matrix_power(x,n):
    """Raises a square matrix to the n-th power.

    Parameters
    ----------
    x : int, float, numpy.ndarray, cas.MS, cas.SX or cas.DM
        Number or array of which the n-th power should be computed.
    n : int or float
        Number defining the exponent.

    """
    
    if not isinstance(n, (int, float)):
        raise TypeError(
            'input n must be of type integer or float - not ' 
            + str(type(n)))
    
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (np.ndarray)):
        y = np.linalg.matrix_power(x_,n)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.mpower(x_,n)
        
    else: 
        raise TypeError(
            'input x must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y

def abs(x):
    """Calculates the absolute value of a number or array."""

    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (int, float, np.ndarray)):
        y = np.absolute(x_)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        y = cas.fabs(x_)
        
    else: 
        raise TypeError(
            'input must be of type integer, float, array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' + str(type(x)))
    
    if isinstance(x, array):
        y = array(y)

    return y
    
def norm(x,order=None):
    """Returns the norm of a vector or matrix.

    Parameters
    ----------
    x : array, numpy.ndarray, cas.MX, cas.SX or cas.DM
        Vector or matrix of which the norm should be calculated.
    order : number or str, optional
        String defining the type of the norm. 
        Posiible values are 1, 2, 'fro' or 'inf'. 
        The default is None.

    """
    if order is not None and order not in [1,2,'fro',inf]:
        raise ValueError(
            'order must be one 1,2, fro or inf - not ' 
            + str(order))
        
    if isinstance(x, array):
        x_ = x._A
    else:
        x_ = x
        
    if isinstance(x_, (np.ndarray)):
        y = LA.norm(x_,order)
        
    elif isinstance(x_, (cas.MX, cas.SX, cas.DM)):
        if order == 1:
            y = cas.norm_1(x_)
        elif order == 2:
            y = cas.norm_2(x_)
        elif order == 'fro':
            y = cas.norm_fro(x_)
        elif order == inf:
            y = cas.norm_inf(x_)
        
    else: 
        raise TypeError(
            'input must be of type array, numpy.ndarray,' 
            + ' casadi.DM, casadi.SX or casadi.MX - not ' 
            + str(type(x)))

    return y

def max(*args):
    """Return the maximal value of the arguments"""
       
    for i in range(len(args)):
        if isinstance(args[i], array):
            args[i] = args[i]._A
        if not isinstance(args[i], (int, float, 
                                    np.ndarray, cas.MX, cas.SX, cas.DM)):
            raise TypeError(
                'input must be of type integer, float, array, numpy.ndarray,' 
                + ' casadi.DM, casadi.SX or casadi.MX - not ' 
                + str(type(args[i])))

    return cas.fmax(*args)

def min(*args):
    """Returns the minimal value of the arguments"""
    
    for i in range(len(args)):
        if isinstance(args[i], array):
            args[i] = args[i]._A
        if not isinstance(args[i], (int, float, 
                                    np.ndarray, cas.MX, cas.SX, cas.DM)):
            raise TypeError(
                'input must be of type integer, float, array, numpy.ndarray,' 
                + ' casadi.DM, casadi.SX or casadi.MX - not ' 
                + str(type(args[i])))

    return cas.fmin(*args)
    
        
if __name__ == '__main__':        
    # a = np.array((1,2,3))
    # a = cas.DM((3,3))
    # print(a[:,0])
    # a = array(a)
    # a[:,0] = np.array([[2,2]])
    # print(a)
    # print(a[:,0])
    
    # b = concatenate((a,a))
    # print(b)
    
    # b = concatenate((a,a),axis=1)
    # print(b)
    
    # print(array.__doc__)
    
    A = zeros(10)
    print(A[10])

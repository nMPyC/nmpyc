#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @author: Jonas Schiessl

import inspect
from functools import wraps

import numpy as np

import nmpyc as mpc

def functionToString(func):
    str_func = inspect.getsource(func)
    name = func.__name__
    return (name, str_func)

def stringToFunction(name,str_func):
    if name == '<lambda>':
        index = str_func.find('lambda')
        func = eval(str_func[index:])
    else:
        exec(str_func)
        func = locals()[name]
    return func

def flat_list(t):
    flat_list = []
    for sub in t:
        for i in range(len(sub)):
            flat_list.append(sub[i])
            
    return flat_list

def mpc_convert(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        # Check if the first argument is 'self' for class methods
        is_method = (inspect.getfullargspec(func)[0][0] == 'self')

        if is_method:
            self_arg = args[0]
            args = args[1:]  # Remove 'self' from the arguments list
        
        # Convert input arguments (x and u) to mpc.array if needed
        converted_args = []
        converted_args.append(args[0])
        for arg in args[1:]:
            if np.isscalar(arg):
                arg = [[arg]]
            elif isinstance(arg, np.ndarray):
                if arg.ndim == 0:
                    arg = [[arg.item()]]  # Convert scalar numpy array to nested list
                else:
                    arg = arg.tolist()  # Convert numpy array to nested list
            converted_args.append(mpc.array(arg))
        
        # Call the original function with the converted arguments
        if is_method:
            y = func(self_arg, *converted_args, **kwargs)
        else:
            y = func(*converted_args, **kwargs)

        # Convert the output (y) to mpc.array if needed
        if np.isscalar(y):
            y = [[y]]
        elif isinstance(y, np.ndarray):
            if y.ndim == 0:
                y = [[y.item()]]  # Convert scalar numpy array to nested list
            else:
                y = y.tolist()  # Convert numpy array to nested list
        return mpc.array(y)

    return wrapper

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

# def mpc_convert(func):
#     sig = inspect.signature(func)
#     params = sig.parameters
#     if inspect.getargspec(func)[0][0] == 'self':
#         if len(params) == 4:
#             def inner(self,t,x,u):
#                 if np.isscalar(x):
#                     x = [[x]]
#                 if np.isscalar(u):
#                     u = [[u]]
#                 x = mpc.array(x)
#                 u = mpc.array(u)
#                 y = func(self,t,x,u)
#                 if np.isscalar(y):
#                     y = [[y]]
#                 return mpc.array(y)
#         elif len(params) == 3:
#             def inner(self,t,x):
#                 if np.isscalar(x):
#                     x = [[x]]
#                 x = mpc.array(x)
#                 y = func(self,t,x)
#                 if np.isscalar(y):
#                     y = [[y]]
#                 return mpc.array(y)
#     else:
#         if len(params) == 3:
#             def inner(t,x,u):
#                 if np.isscalar(x):
#                     x = [[x]]
#                 if np.isscalar(u):
#                     u = [[u]]
#                 x = mpc.array(x)
#                 u = mpc.array(u)
#                 y = func(t,x,u)
#                 if np.isscalar(y):
#                     y = [[y]]
#                 return mpc.array(y)
#         elif len(params) == 2:
#             def inner(t,x):
#                 if np.isscalar(x):
#                     x = [[x]]
#                 x = mpc.array(x)
#                 y = func(t,x)
#                 if np.isscalar(y):
#                     y = [[y]]
#                 return mpc.array(y)
#     return inner

def mpc_convert(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        sig = inspect.signature(func)
        params = sig.parameters
        if inspect.getargspec(func)[0][0] == 'self':
            if len(params) == 4:
                def inner(self,t,x,u):
                    if np.isscalar(x):
                        x = [[x]]
                    if np.isscalar(u):
                        u = [[u]]
                    x = mpc.array(x)
                    u = mpc.array(u)
                    y = func(self,t,x,u)
                    if np.isscalar(y):
                        y = [[y]]
                    return mpc.array(y)
            elif len(params) == 3:
                def inner(self,t,x):
                    if np.isscalar(x):
                        x = [[x]]
                    x = mpc.array(x)
                    y = func(self,t,x)
                    if np.isscalar(y):
                        y = [[y]]
                    return mpc.array(y)
        else:
            if len(params) == 3:
                def inner(t,x,u):
                    if np.isscalar(x):
                        x = [[x]]
                    if np.isscalar(u):
                        u = [[u]]
                    x = mpc.array(x)
                    u = mpc.array(u)
                    y = func(t,x,u)
                    if np.isscalar(y):
                        y = [[y]]
                    return mpc.array(y)
            elif len(params) == 2:
                def inner(t,x):
                    if np.isscalar(x):
                        x = [[x]]
                    x = mpc.array(x)
                    y = func(t,x)
                    if np.isscalar(y):
                        y = [[y]]
                    return mpc.array(y)
        return inner(*args, **kwargs)
    return wrapper

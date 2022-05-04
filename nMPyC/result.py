#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  5 18:41:00 2022

@author: Jonas Schiessl
"""

import numpy as np

import matplotlib.pyplot as plt

from collections.abc import Iterable
import dill

import nMPyC as mpc

class result:
    def __init__(self, x0, t0, h, N, K):
        
        self._t0 = t0
        self._x0 = x0
        
        self._t = t0
        
        self._h = h
        self._N = N
        self._K = K
        self._KK = K
        
        self._x_cl = []
        self._u_cl = []
        self._l_cl = []
        self._t_cl = []
        
        self._x_ol = []
        self._u_ol = []
        self._l_ol = []
        self._t_ol = []
        
        self._solver = None
        self._success = True
        self._error = None
        self._time = None
        self._time_ol = []
        
        self._default_kwargs = dict(xk = None,
                                    uk = None,
                                    iters = None,
                                    show_ol = False,
                                    phase1 = None,
                                    phase2 = None,
                                    dpi = 500,
                                    figsize = [9.6, 7.2],
                                    usetex = True,
                                    fontsize = 20,
                                    grid = True,
                                    linewidth = 2.,
                                    show_legend = True)
        
    @property 
    def t0(self):
        return self._t0
    
    @property 
    def x0(self):
        return self._x0
    
    @property 
    def sampling_rate(self):
        return self._h
    
    @property 
    def N(self):
        return self._N
    
    @property
    def sucessfull_itertaions(self):
        return self._KK
    
    @property 
    def x_cl(self):
        return self._x_cl
    
    @property 
    def u_cl(self):
        return self._u_cl
    
    @property 
    def l_cl(self):
        return self._l_cl
    
    @property 
    def t_cl(self):
        return self._t_cl
    
    @property 
    def x_ol(self):
        return self._x_ol
    
    @property 
    def u_ol(self):
        return self._u_ol
        
    @property 
    def l_ol(self):
        return self._l_ol
    
    @property
    def t_ol(self):
        return self._t_ol
    
    @property 
    def solver(self):
        return self._solver
    
    @property 
    def succes(self):
        return self._succes
    
    @property 
    def error(self):
        return self._error
    
    @property 
    def ellapsed_time(self):
        return self._time
    
    @property 
    def ellapsed_time_per_itertaion(self):
        return self._time_ol
        
    def __str__(self):
        string = ''
        string += 'solver: ' + str(self._solver) + '\n'
        string += 'success: ' + str(self._success) + '\n'
        string += ('succesfull iterations: ' 
                   + str(self._K) 
                   + ' of ' 
                   + str(self._KK) 
                   + '\n')
        string += 'ellapsed time: ' + str(self._time) + '\n'
        string += 'x0: ' + str(self._x0.flatten()) + '\n'
        string += 't0: ' + str(self._t0) + '\n'
        string += 'h: ' + str(self._h) + '\n'
        string += 'N: ' + str(self._N) + '\n'
        string += 'x_cl: ' + str(self._x_cl) + '\n'
        string += 'u_cl: ' + str(self._u_cl) + '\n'
        string += 'l_cl: ' + str(self._l_cl.flatten())
        return string
        
    def _add_iteration(self, x, u, l):
        
        t = mpc.array(np.arange(self._t, 
                                self._t + (self._N + 1)*self._h - (self._h/2), 
                                self._h))
        self._t_ol += [t]
        
        self._t += self._h
        
        self._x_ol += [x]
        self._u_ol += [u]
        self._l_ol += [l]
        
    def _init_cl(self, t_end, x_end):
        
        for i in range(self._K):
            self._x_cl += [list(mpc.convert(self._x_ol[i][:,0], 
                                           'numpy').flatten())]
            self._u_cl += [list(mpc.convert(self._u_ol[i][:,0], 
                                           'numpy').flatten())]
            self._t_cl += [self._t_ol[i][0]]
            self._l_cl += [self._l_ol[i][0]]
        
        self._x_cl += [list(mpc.convert(x_end, 'numpy').flatten())]
        self._t_cl += [t_end]
        
        self._x_cl = mpc.array(self._x_cl).T
        self._u_cl = mpc.array(self._u_cl).T
        self._t_cl = mpc.array(self._t_cl)
        self._l_cl = mpc.array(self._l_cl)
        
        try:
            self._default_kwargs['xk'] = range(1, len(self._x_cl[:,0]) + 1)
            self._default_kwargs['uk'] = range(1, len(self._u_cl[:,0]) + 1)
            self._default_kwargs['iters'] = range(1, self._K + 1)
        except:
            self._default_kwargs['xk'] = []
            self._default_kwargs['uk'] = []
            
            
    def show_errors(self):
        if self._success:
            print('No error occured during the MPC-Loop.')
        else:
            print('An error occured during itertaion ' 
                  + str(self._K + 1) 
                  + ' of ' + str(self._KK) + ':')
            print(self._error)
        
    def plot(self, *args, **kwargs):
        
        if self._default_kwargs['xk'] == []:
            print('Cant plot anything because there is' 
                  + ' no successfull itertaion stored.')
            return
        
        for item in kwargs.keys():
            if item not in self._default_kwargs.keys():
                raise KeyError()
                
        options = self._default_kwargs.copy()
        options.update(kwargs)
        
        if isinstance(options['xk'], int):
            options['xk'] = [options['xk']]
        if not isinstance(options['xk'], Iterable):
            raise TypeError('xk must be an integer or iterable')
            
        if isinstance(options['uk'], int):
            options['uk'] = [options['uk']]
        if not isinstance(options['uk'], Iterable):
            raise TypeError('uk must be an integer or iterable')
            
        if isinstance(options['iters'], int):
            options['iters'] = [options['iters']]
        if not isinstance(options['iters'], Iterable):
            raise TypeError('iters must be an integer or iterable')
        
        for item in options['xk']:
            if item not in self._default_kwargs['xk']:
                raise ValueError(
                    'Value ' + str(item) + ' of xk is out of range.')
                
        for item in options['uk']:
            if item not in self._default_kwargs['uk']:
                raise ValueError(
                    'Value ' + str(item) + ' of uk is out of range.')
                
        for item in options['iters']:
            if item not in self._default_kwargs['iters']:
                raise ValueError(
                    'Value ' + str(item) + ' of iters is out of range.')
        
        dimsx = [x-1 for x in options['xk']] 
        dimsu = [x-1 for x in options['uk']] 
        iters = [x-1 for x in options['iters']]
        
        plt.rcParams.update({
            "text.usetex": options['usetex'],
            "font.family": "sans-serif",
            "font.sans-serif": ["Helvetica"],
            "font.size": options['fontsize']})
        
        if len(args) == 0: 
            key = 'cl'
        elif len(args) == 1:
            if args[0] == 'state':
                key = 'x_'
            elif args[0] == 'control':
                key = 'u_'
            elif args[0] == 'phase':
                key = 'phase_'
            elif args[0] == 'cost':
                key = 'l_'    
            else: raise ValueError()
            
            if options['show_ol']:
                key += 'ol'
            else:
                key += 'cl'
        
        if key == 'cl':
            
            if options['usetex']:
                ylabelx = r'\textbf{state}'
                ylabelu = r'\textbf{control}'
                xlabel = r'\textbf{time}'
                title = r'\textbf{Closed Loop}'
            else:
                ylabelx = 'state'
                ylabelu = 'control'
                xlabel = 'time'
                title = 'Close Loop'
            
            fig, (ax1, ax2) = plt.subplots(2, 1, tight_layout=True, 
                                           sharex=True, 
                                           figsize=options['figsize'], 
                                           dpi=options['dpi'])
            fig.subplots_adjust(hspace = 0.2)
            
            for i in dimsx:
                if options['usetex']:
                    label = r'$x_' + str(i + 1) + '$'
                else:
                    label = 'x_' + str(i + 1)
                ax1.plot(self._t_cl, self._x_cl[i,:], 
                         label=label, linewidth=options['linewidth'])
            
            ax1.set_ylabel(ylabelx)
            ax1.grid(options['grid'])
            if options['show_legend']:
                ax1.legend()

            for i in dimsu:
                if options['usetex']:
                    label = r'$u_' + str(i + 1) + '$'
                else:
                    label = 'u_' + str(i + 1)
                ax2.step(self._t_cl[:-1], self._u_cl[i, :], where='post', 
                         label=label, linewidth=options['linewidth'])
            
            ax2.set_ylabel(ylabelu)
            ax2.grid(options['grid'])
            if options['show_legend']:
                ax2.legend()
            
            ax2.set_xlabel(xlabel)
            
            fig.suptitle(title)
            fig.align_ylabels((ax1, ax2))

        elif key == 'x_cl':
            
            if options['usetex']:
                ylabelx = r'\textbf{state}'
                xlabel = r'\textbf{time}'
                title = r'\textbf{Closed Loop States}'
            else:
                ylabelx = 'state'
                xlabel = 'time'
                title = 'Close Loop States'
            
            fig, (ax1) = plt.subplots(1, 1, tight_layout=True, 
                                      figsize=options['figsize'], 
                                      dpi=options['dpi'])
            fig.subplots_adjust(hspace = 0.5)
            
            for i in dimsx:
                if options['usetex']:
                    label = r'$x_' + str(i + 1) + '$'
                else:
                    label = 'x_' + str(i + 1)
                ax1.plot(self._t_cl, self._x_cl[i, :], label=label, 
                         linewidth=options['linewidth'])
            
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabelx)
            if options['show_legend']:
                ax1.legend()
            ax1.grid(options['grid'])
            
            fig.suptitle(title)
            plt.show()
            
        elif key == 'u_cl':
            
            if options['usetex']:
                ylabelu = r'\textbf{control}'
                xlabel = r'\textbf{time}'
                title = r'\textbf{Closed Loop Controls}'
            else:
                ylabelu = 'control'
                xlabel = 'time'
                title = 'Closed Loop Controls'
            
            fig, (ax1) = plt.subplots(1, 1, tight_layout = True, 
                                      figsize = options['figsize'], 
                                      dpi = options['dpi'])
            
            for i in dimsu:
                if options['usetex']:
                    label = r'$u_' + str(i + 1) + '$'
                else:
                    label = 'u_' + str(i + 1)
                ax1.step(self._t_cl[:-1], self._u_cl[i, :], where = 'post', 
                         label = label, linewidth = options['linewidth'])
            
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabelu)
            if options['show_legend']:
                ax1.legend()
            ax1.grid(options['grid'])
            
            fig.suptitle(title)
            plt.show()
            
        elif key == 'x_ol':
            
            nx = len(self._x_cl[:,0])
            
            if options['usetex']:
                xlabel = r'\textbf{time}'
                title = r'\textbf{Open Loop States}'
            else:
                xlabel = 'time'
                title = 'Open Loop States'
            
            fig, ax = plt.subplots(len(dimsx), 1, sharex=True, 
                                   tight_layout = True, 
                                   figsize = options['figsize'], 
                                   dpi = options['dpi'])
            fig.subplots_adjust(hspace = 0.2)
            
            if nx == 1: 
                ax = (ax,)
            
            j = 0
            for k in dimsx:
                if options['usetex']:
                    label = r'$x_' + str(k + 1) + '$'
                    ylabelx = r'$\mathbf{x_' + str(k + 1) + '}$'
                else:
                    label = 'x_' + str(k + 1)
                    ylabelx = 'x_' + str(k + 1) 
                for i in iters:
                    ax[j].plot(self._t_ol[i], self._x_ol[i][k, :], 
                               label = label, color = 'grey', 
                               linewidth = options['linewidth'])
                
                ax[j].plot(self._t_cl, self._x_cl[k, :], 'o-', alpha = 0.3, 
                           label = label, color = 'b', 
                           linewidth = options['linewidth'])
                
                ax[j].set_ylabel(ylabelx)
                ax[j].grid(options['grid'])
                j += 1
                
            ax[-1].set_xlabel(xlabel)
            
            fig.suptitle(title)
            fig.align_ylabels(ax)
            plt.show()
            
        elif key == 'u_ol':
            
            nu = len(self._u_cl[:, 0])
            
            if options['usetex']:
                xlabel = r'\textbf{time}'
                title = r'\textbf{Open Loop States}'
            else:
                xlabel = 'time'
                title = 'Open Loop Controls'
            
            fig, ax = plt.subplots(len(dimsu), 1, sharex = True, 
                                   tight_layout = True, 
                                   figsize = options['figsize'], 
                                   dpi = options['dpi'])
            fig.subplots_adjust(hspace = 0.2)
            
            if nu == 1: 
                ax = (ax,)
            
            j = 0
            for k in dimsu:
                if options['usetex']:
                    label = r'$u_' + str(k + 1) + '$'
                    ylabelu = r'$\mathbf{u_' + str(k + 1) + '}$'
                else:
                    label = 'u_' + str(k + 1)
                    ylabelu = 'u_' + str(k + 1) 
                for i in iters:
                    ax[j].step(self._t_ol[i][:-1], self._u_ol[i][k, :], 
                               where = 'post', label = label, color = 'grey', 
                               linewidth = options['linewidth'])
                
                ax[j].step(self._t_cl[:-1], self._u_cl[k, :], 'o-', alpha = 0.3, 
                           where = 'post', label = label, color = 'b', 
                           linewidth = options['linewidth'])
                
                ax[j].set_ylabel(ylabelu)
                ax[j].grid(options['grid'])
                j += 1
                
            ax[-1].set_xlabel(xlabel)
            
            fig.suptitle(title)
            fig.align_ylabels(ax)
            plt.show()
            
        elif key == 'l_cl':
            
            if options['usetex']:
                xlabel = r'\textbf{time}'
                ylabel = r'$\mathbf{\ell (x,u)}$'
                title = r'\textbf{Closed Loop Costs}'
            else:
                xlabel = 'time'
                ylabel = 'l(x,u)'
                title = 'Closed Loop Costs'
                    
            fig, (ax1) = plt.subplots(1, 1, tight_layout = True, 
                                      figsize = options['figsize'], 
                                      dpi = options['dpi'])
            
            ax1.plot(self._t_cl[:-1], self._l_cl, 
                     linewidth = options['linewidth'])
            
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabel)
            ax1.grid(options['grid'])
            
            fig.suptitle(title)
            plt.show()
                    
        elif key == 'l_ol':
            
            if options['usetex']:
                xlabel = r'\textbf{time}'
                ylabel = r'$\mathbf{\ell (x,u)}$'
                title = r'\textbf{Open Loop Costs}'
            else:
                xlabel = 'time'
                ylabel = 'l(x,u)'
                title = 'Open Loop Costs'
                    
            fig, (ax1) = plt.subplots(1, 1, tight_layout = True, 
                                      figsize = options['figsize'], 
                                      dpi = options['dpi'])
            
            for i in iters:
                ax1.plot(self._t_ol[i][:-1], self._l_ol[i], color='grey')
            
            ax1.plot(self._t_cl[:-1], self._l_cl, 'o-', alpha = 0.3, 
                     color = 'b', linewidth = options['linewidth'])
            
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabel)
            ax1.grid(options['grid'])
            
            fig.suptitle(title)
            plt.show()
            
        elif key == 'phase_cl':
            
            phase = (kwargs['phase1'], kwargs['phase2'])
            
            if options['usetex']:
                xlabel = r'$\mathbf{' + phase[0] + '}$'
                ylabel = r'$\mathbf{' + phase[1] + '}$'
                title = (r'$\mathbf{' + phase[0] + '-' + phase[1] 
                         + '~Closed~Loop' + '~Portrait' + '}$')
            else:
                xlabel = phase[0]
                ylabel = phase[1]
                title = (phase[0] + '-' + phase[1] 
                         + ' Closed Loop' + ' Portrait')
            
            if phase[0][0] == 'u' or phase[1][0] == 'u':
                T = self._K
            else:
                T = self._K + 1
            
            x = []
            
            for i in range(2):
                if phase[i][1] != '_': raise ValueError()
                if phase[i][0] == 'x':
                    k = int(phase[i][2:])
                    x += [self._x_cl[k-1, :T]]
                if phase[i][0] == 'u':
                    k = int(phase[i][2:])
                    x += [self._u_cl[k-1, :]]
                    
            fig, (ax1) = plt.subplots(1, 1, tight_layout = True, 
                                      figsize = options['figsize'], 
                                      dpi = options['dpi'])
            
            ax1.plot(x[0], x[1], linewidth = options['linewidth'])
            
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabel)
            ax1.grid(options['grid'])
            
            fig.suptitle(title)
            plt.show()
                    
        elif key == 'phase_ol':
             
            phase = (kwargs['phase1'], kwargs['phase2'])
            
            if options['usetex']:
                xlabel = r'$\mathbf{' + phase[0] + '}$'
                ylabel = r'$\mathbf{' + phase[1] + '}$'
                title = (r'$\mathbf{' + phase[0] + '-' + phase[1] 
                         + '~Open~Loop' + '~Portrait' + '}$')
            else:
                xlabel = phase[0]
                ylabel = phase[1]
                title = phase[0] + '-' + phase[1] + ' Open Loop' + ' Portrait'
            
            x = []
            
            if phase[0][0] == 'u' or phase[1][0] == 'u':
                T1 = self._N
                T2 = self._K
            else:
                T1 = self._N + 1
                T2 = self._K + 1
            
            fig, (ax1) = plt.subplots(1, 1, tight_layout = True, 
                                      figsize = options['figsize'], 
                                      dpi = options['dpi'])
            
            for j in iters:
                for i in range(2):
                    if phase[i][1] != '_': 
                        raise ValueError()
                    if phase[i][0] == 'x':
                        k = int(phase[i][2:])
                        x += [self._x_ol[j][k - 1, :T1]]
                    if phase[i][0] == 'u':
                        k = int(phase[i][2:])
                        x += [self.u_ol[j][k - 1, :]]
                
                ax1.plot(x[0], x[1], color = 'grey', 
                         linewidth = options['linewidth'])
                x = []
                        
            for i in range(2):
                if phase[i][0] == 'x':
                    k = int(phase[i][2:])
                    x += [self._x_cl[k - 1, :T2]]
                if phase[i][0] == 'u':
                    k = int(phase[i][2:])
                    x += [self._u_cl[k - 1, :]]
            
            ax1.plot(x[0], x[1], 'o-', alpha = 0.3, color = 'b', 
                     linewidth = options['linewidth'])
            
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabel)
            ax1.grid(options['grid'])
            
            fig.suptitle(title)
            plt.show()
            
        return fig
    
    def save(self, path):
        
        with open(path, "wb") as output_file:
            dill.dump(self, output_file, -1)
    
    @classmethod
    def load(cls, path):
        
        try:
            with open(path, "rb") as input_file:
                e = dill.load(input_file)
        except:
            raise Exception(
                'Can not load model from file. File not readable!')
            
        if not isinstance(e, result):
            raise Exception(
                'Can not load model from file. File does not cotain a result!')
            
        return e
        
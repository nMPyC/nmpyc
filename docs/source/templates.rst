Templates
==========

In addition to the examples, we also provide templates to facilitate the implementation.

To take advantage of the different structures of the problems, we have implemented templates for the following problem types.

.. toctree::

   template/timevariant
   template/autonomous
   template/lqp


.. note::

   Any problem, whether nonlinear, linear, autonomous, or time-varying, can be initialized as a 
   nonlinear time-varying optimal control problem. 
   Therefore, you can always fall back on such an implementation.
   However, if you know the structure of your problem and this is to be exploited by the program 
   in order to possibly speed up the simulation, it is necessary to initialize the problem as such. 

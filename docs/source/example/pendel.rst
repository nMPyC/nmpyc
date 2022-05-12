Inverted Pendulum
==================

We consider the mechanical model of an inverted rigid pendulum mounted on a carriage.

The control :math:`u` is the acceleration of the carriage. 
By means of physical laws an "exact" differential equation model can be derived.
However, since in our case we want to obtain a linear quadratic problem, 
we linearize the differential equation at the origin.
Thus we obtain the system dynamics defined by 

.. math::

   \left(\begin{array}{cccc} 
      0 & 1 & 0 & 0 \\
      g & -k & 0 & 0 \\
      0 & 0 & 0 & 1 \\
      0 & 0 & 0 & 0
   \end{array}\right)


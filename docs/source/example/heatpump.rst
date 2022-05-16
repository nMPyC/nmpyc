Heat Pump
==========

This example is a home heating system that involves the optimal control of a small heat pump coupled to a floor heating system. The corresponding dynamic model was used in :cite:p:`Logist2010` and is given by

.. math::
   :nowrap:
   
   \begin{align}
	\dot{x_1} &= \dfrac{-k_{WR}}{\rho_W c_W V_H}x_1 + \dfrac{k_{WR}}{\rho_W c_W V_H}x_2 + \dfrac{1}{\rho_W c_W V_H}u\\
	\dot{x_2}&= \dfrac{k_{WR}}{k_G \tau_G}x_1 -\dfrac{k_{WR}+k_G}{k_G\tau_G}x_2 +\dfrac{1}{\tau_G} T_{\text{amb}},
   \end{align}
   
where :math:`x_1` denotes the temperature of the water returning from the heating, :math:`x_2` denotes the room temperature and :math:`u` is the heat supplied from the heat pump to the floor. Further, the ambient temperature

.. math::
   T_\text{amb}(t) = 2-5 + 7.5 \sin\left(\frac{2\pi t}{t_f}-\frac \pi 2\right)
   
describes a sinusoidal disturbance from the outside temperature where :math:`t_f = 24`. The remaining constants are summarized in the table below.

============================================================ ================  ============  ================
   Reactor constants
-------------------------------------------------------------------------------------------------------------
            \                             \                                      value          unit
============================================================ ================  ============  ================
density of the water                                          :math:`\rho_W`     997          :math:`kg/m^3`
specific heat capacity of water                               :math:`c_W`        4.1851       :math:`J/kgK`     
volume of the water                                           :math:`V_H`        7.4          :math:`m^3` 
thermal conductivity between water and the room               :math:`k_{WR}`     510          :math:`W/K`
thermal conductivity between the room and the environment     :math:`k_G`        125          :math:`W/K`
thermal time constant of the room                             :math:`\tau_G`     260          :math:`s`
============================================================ ================  ============  ================

In the heating system the conflict between energy and thermal comfort arises. Thus, the stage cost read

.. math::
   :nowrap:
   
   \begin{align*}
	\ell(x,u)&=\frac u P_{\max} + (x_2-T_\text{ref})^2,
   \end{align*}
   
where :math:`P_{\max} = 15000 (W)` is the maximal power of the heating pump and :math:`T_\text{ref} = 22^{°} C` is the desired temperature of the room. The reference temperature :math:`T_\text{ref}` can be selected differently -- depending on the thermal comfort.


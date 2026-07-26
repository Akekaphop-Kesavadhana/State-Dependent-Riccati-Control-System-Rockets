# State Dependent Riccati Control System Rockets 🚀

A MATLAB codebase is presented to simulate a Six-Degree-of-Freedom (6-DoF) 
model rocket which will be used to benchmark a nonlinear control system. 
The nonlinear control system that will be employed will be based around 
a Nonlinear Quadratic Regulator (NQR), more specifically a State-Dependent 
Riccati Equation (SDRE) control system. 

In linear optimal control, a Linear Quadratic Regulator (LQR) is used to create a porportional 
feedback control law with gains that are tuned by solving an Algebraic Riccati Equation (ARE).
The ARE is derived by minimizeing a state and control cost-functional that is defined by a classic
Calculus of Variations optimization problem called the Bolza Problem. Such a problem is defined as an
integral involving a running cost as the integrand and a final terminal cost. When the 
running cost (i.e. integrand) is choosen to depend on both the states and control inputs to the 
system specified as x^TQx and u^TRu respectively then the running cost is known to be of the quadratic form.
By designing an optimal control input u* such that the running cost is zero for all time then that 
implies that the system states will traverse some optimal trajectory x* as time advances.

Written by hand.

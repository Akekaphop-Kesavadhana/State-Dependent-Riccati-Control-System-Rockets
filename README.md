# State Dependent Riccati Control System Rockets 🚀

A MATLAB codebase is presented to simulate a Six-Degree-of-Freedom (6-DoF) 
model rocket which will be used to benchmark a nonlinear control system. 
The nonlinear control system that will be employed will be based around 
a Nonlinear Quadratic Regulator (NQR), more specifically a State-Dependent 
Riccati Equation (SDRE) control system. 

Recalling from optimal control, a Linear Quadratic Regulator (LQR) is used to create a proportional 
feedback control policy with gains that are tuned by solving an Algebraic Riccati Equation (ARE).
The ARE is derived by minimizing a state and control cost-functional that is defined by a classic
Calculus of Variations optimization problem called the Bolza Problem. Such a problem is written as a sum
involving an integral with a running cost as the integrand and a final terminal cost. When the running cost 
(i.e. integrand) is choosen to depend on both the states and control inputs to the system specified as 
x^TQx and u^TRu respectively then the running cost is known to be of the quadratic form. By designing an 
optimal control input u* such that the running cost is zero for all time then that implies that the system 
states will traverse some optimal trajectory x*. Therefore, the optimal states are not unique and thus 
allow the designer to modify u such that the trajectories remain within certain tolerances or requirements as 
dictated by the project requirements. The problem that must now be solved is given simply from the proportional 
feedback policy u = -K*x where K, the Kalman Gain Matrix is R^(-1)(B)^TP*x. Observing that the control input weighting 
matrix R and control input matrix B along with the state vector x are known then the Riccati Matrix P shall be 
the only term that needs to be obtained. Hence, the ARE will be solved backwards in time to determine P and 
then passed to the proportional policy. 

Written by Akekaphop Kesavadhana, University of Utah Aerospace Club President.

clc; clear; close all;


MotorData = readmatrix('AeroTech_HP-H135W.csv');

% Measured motor thrust curve
t_thrust = MotorData(:,1);   % thrust-curve time samples
F_thrust = MotorData(:,2);   % corresponding thrust values

% Simulation time vector
t_sim = 0:0.001:5;

% Interpolate thrust onto simulation time vector
F_sim = interp1(t_thrust, F_thrust, t_sim, 'makima', 0);

plot(t_sim,F_sim);

%%

% syms ixx iyx iyy izx izy izz P Q R L M N 
% ixy = 0; iyz = 0; ixz = 0;
% I = [ixx -ixy -ixz; 
%     -ixy  iyy -iyz; 
%     -ixz -iyz  izz];
% Iinv = inv(I)

% Hx = P*ixx - Q*ixy - R*ixz
% Hy = Q*iyy - R*iyz - P*ixy
% Hz = R*izz - P*ixz - Q*iyz
% v  = [Q*Hz - R*Hy;
%       R*Hx - P*Hz;
%       P*Hy - Q*Hx];
% moment = [L; M; N];
% omega_dot = Iinv*(moment - v)

% A1 = L - Q*R*izz + P*Q*ixz + (Q)^2*iyz + Q*R*iyy - (R)^2*iyz - P*R*ixy;
% A2 = M - P*R*ixx + Q*R*ixy + (R)^2*ixz + P*R*izz - (P)^2*ixz - P*Q*iyz;
% A3 = N - P*Q*iyy + P*R*iyz + (P)^2*ixy + P*Q*ixx - (Q)^2*ixy - Q*R*ixz;
% A  = [A1; A2; A3];
% omega_dot = Iinv*A

% xdot = A(x)x + B(x,u)u
% u = [del1 del2 del3 del4]
% x = [Xe Ye Ze P Q R]
% xdot = [Xed Yed Zed Pd Qd Rd]

% [Xedd;
%  Yedd;
%  Zedd] = CeB * [ud;
                % ud;
                % wd];

% [Xedd;
%  Yedd;
%  Zedd] = CeB * [Rv - Qw + Fx/m;
                % Pw - Ru + Fy/m;
                % Qu - Pv + Fz/m];   

% x1 = Xed; x1d = Xedd;
% x2 = Yed; x2d = Yedd;
% x3 = Zed; x3d = Zedd;

% [x1d;
%  x2d;
%  x3d] = [Rv - Qw + Fx/m;
         % Pw - Ru + Fy/m;
         % Qu - Pv + Fz/m];   

% Pdot = ((Q*R*(iyy - izz))/ixx) + (1/ixx)*(del1 + del2 + del3 + del4)/4;
% Qdot = ((P*R*(izz - ixx))/iyy) + (1/iyy)*(del1 + del2)/2;
% Rdot = ((P*Q*(ixx - iyy))/izz) + (1/izz)*(del3 + del4)/2;

% uA = (del1 + del2 + del3 + del4)/4;
% uE = (del1 + del2)/2;
% uR = (del3 + del4)/2;

% u = [del1; del2; del3; del4]


% x = [P Q R];

% A(x) = [0             R*(iyy/ixx)   -Q*(izz/ixx);
%         R*(izz/iyy)   0             -R*(ixx/iyy);
%         Q*(ixx/izz)  -P*(iyy/izz)    0;

% B(x,u) = [(1/(4*ixx)) (1/(4*ixx)) (1/(4*ixx)) (1/(4*ixx));
%           (1/(2*iyy)) (1/(2*iyy))      0           0;
%           0            0          (1/(2*izz)) (1/(2*izz))];

% xdot = A(x)x + B(x,u)u

% u = -K*x

% Q = diag([1 10 100]); 
% R = diag([10 10 10]);

% Obtain numerical entries for A(x) and B(x,u) at each time step A(x(1)), A(x(2))...
% Solve State Dependent Riccat Equation for P(x,u) using ARE(A,B,Q,R) i.e ARE(A(x(1)),B(x(1),u(1)),Q,R)...
% Use to get the control vector u(x) = -R^(-1)*B'(x,u)*P(x,u)*x i.e u(x(1)) = -R^(-1)*B^T(x(1),u(1))*P(x(1),u(1))*x(1)

ti = 0; dt = 0.01; tf = 5;
tsim   = ti:dt:tf;

x0     = [0.05; -0.02; 0.03];
[t, x] = ode45(@(t,x) EQ(t,x), tsim, x0);
P = x(:,1); Q = x(:,2); R = x(:,3);

u = zeros(length(t),4);
for i = 1:length(t)
    u(i,:) = sdreControl(t,x(i,:)');
end
%% -------------- %% 

figure(1);
subplot(1,2,1);
plot(t,P); hold on; 
plot(t,Q);
plot(t,R);
legend('P - Roll Rate','Q - Pitch Rate','R - Yaw Rate');
grid on;
subplot(1,2,2);
plot(t,u);
legend('del1','del2','del3','del4');
grid on;

%% -------------- %% 
function dxdt = EQ(t,x)
    P = x(1); Q = x(2); R = x(3);
    ixx = 1; iyy = 1; izz = 1;
    A = [0             R*(iyy/ixx)   -Q*(izz/ixx);
         R*(izz/iyy)   0             -R*(ixx/iyy);
         Q*(ixx/izz)  -P*(iyy/izz)    0];
    
    B = [(1/(4*ixx)) (1/(4*ixx)) (1/(4*ixx)) (1/(4*ixx));
         (1/(2*iyy)) (1/(2*iyy))      0           0;
         0            0          (1/(2*izz)) (1/(2*izz))];
    u = sdreControl(t,x);
    dxdt = A*x + B*u;
end
%% -------------- %% 
function u = sdreControl(t,x)
    u = zeros(4,length(t));
    for teval = 1:length(t)
        [A,B,Q,R] = Aeval(x);
        [~,K,~] = icare(A,B,Q,R);
        u = -K*x;
        % u = -R^(-1)*B'*P*x
    end
end
%% -------------- %% 
function [A,B,q,r] = Aeval(StateVector)
    P = StateVector(1); Q = StateVector(2); R = StateVector(3);
    ixx = 1; iyy = 1; izz = 1;
    A = [0             R*(iyy/ixx)   -Q*(izz/ixx);
         R*(izz/iyy)   0             -R*(ixx/iyy);
         Q*(ixx/izz)  -P*(iyy/izz)    0];                      % A(x)
    B = [(1/(4*ixx)) (1/(4*ixx)) (1/(4*ixx)) (1/(4*ixx)); 
         (1/(2*iyy)) (1/(2*iyy))      0           0;
         0            0          (1/(2*izz)) (1/(2*izz))];     % B(x,u)
    % q = diag([2*Q P R]);                                      % Q(x)
        q = diag([100 50 100]);                                      % Q(x)

    r = diag([2 2 2 2]);                                       % R(x)
end



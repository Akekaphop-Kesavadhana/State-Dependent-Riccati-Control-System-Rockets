clc; clear; close all;

Xinertial = 0; Yinertial = 0; Zinertial = 0; % Initial inertial position [m]
u0 = 0; v0 = 0; w0 = 0;                      % Initial body translational velocities [m/s]
P0 = 0; Q0 = 0; R0 = 0;                      % Initial body angular velocities [rad/s]
phi0deg   = 0;                               % Initial Euler roll angle [deg]
theta0deg = 50;                              % Initial Euler pitch angle [deg]
psi0deg   = 0;                               % Initial Euler yaw angle [deg]

% Euler angle degrees to radian conversion
phi0   = deg2rad(phi0deg);                   % Initial Euler roll angle [rad]
theta0 = deg2rad(theta0deg);                 % Initial Euler pitch angle [rad]
psi0   = deg2rad(psi0deg);                   % Initial Euler yaw angle [rad]

% Euler angles to quaternion conversion [q0 q1 q2 q3]
q0 = cos(psi0/2)*cos(theta0/2)*cos(phi0/2) - sin(psi0/2)*sin(theta0/2)*sin(phi0/2); % Initial quaternion q0
q1 = sin(theta0/2)*sin(phi0/2)*cos(psi0/2) + sin(psi0/2)*cos(theta0/2)*cos(phi0/2); % Initial quaternion q1
q2 = sin(theta0/2)*cos(psi0/2)*cos(phi0/2) - sin(psi0/2)*sin(phi0/2)*cos(theta0/2); % Initial quaternion q2
q3 = sin(phi0/2)*cos(psi0/2)*cos(theta0/2) + sin(psi0/2)*sin(theta0/2)*cos(phi0/2); % Initial quaternion q3

% eul  = [phi0 theta0 psi0];
% qZYX = eul2quat(eul);
% q    = [q0; q1; q2; q3];

opts = odeset('Events', @stopEvent);                                                % Simulation stop event for when vehicle makes contact with ground
ti = 0; dt = 0.001; tf = 20;                                                        % Simulation time paramters
tsim   = ti:dt:tf;                                                                  % Simulation time vector
x0     = [u0; v0; w0; P0; Q0; R0; Xinertial; Yinertial; Zinertial; q0; q1; q2; q3]; % Initial condition vector 
[t, x] = ode45(@(t,x)rocket_6DoFEquationsOfMotion(t,x), tsim, x0, opts);            % Numerical solver 
 u  = x(:,1);   v = x(:,2);   w = x(:,3);             
 P  = x(:,4);   Q = x(:,5);   R = x(:,6);
Xe  = x(:,7);  Ye = x(:,8);  Ze = x(:,9);
q0  = x(:,10); q1 = x(:,11); q2 = x(:,12); q3 = x(:,13); 

eulZYX   = quat2eul([q0 q1 q2 q3],'ZYX')*(180/pi);
psi = eulZYX(:,1); theta = eulZYX(:,2); phi = eulZYX(:,3);
altitude = -Ze;
qnorm    = sqrt(q0.^2 + q1.^2 + q2.^2 + q3.^2);

%=====================% Post-processing and plotting %=====================%
figure(1);
plot3(Xe, Ye, altitude, LineWidth=3);
xlabel('$X_i - Downrange (m)$', 'Interpreter','latex'); 
ylabel('$Y_i - Crossrange (m)$', 'Interpreter','latex');
zlabel('$Z_i - Altitude (m)$', 'Interpreter','latex'); grid on;
daspect([1 1 8]);        
pbaspect([1 1 2]);       

figure(2);
subplot(2,3,1);
plot(t,u); grid on; xlabel('time (s)'); ylabel('u'); title('Linear Velocity u'); xlim([0 t(end)]);
subplot(2,3,2);
plot(t,v); grid on; xlabel('time (s)'); ylabel('v'); title('Linear Velocity v'); xlim([0 t(end)]);
subplot(2,3,3);
plot(t,w); grid on; xlabel('time (s)'); ylabel('w'); title('Linear Velocity w'); xlim([0 t(end)]);
subplot(2,3,4);
plot(t,P); grid on; xlabel('time (s)'); ylabel('P'); title('Roll Rate P'); xlim([0 t(end)]); 
subplot(2,3,5);
plot(t,Q); grid on; xlabel('time (s)'); ylabel('Q'); title('Pitch Rate Q'); xlim([0 t(end)]);
subplot(2,3,6);
plot(t,R); grid on; xlabel('time (s)'); ylabel('R'); title('Yaw Rate R'); xlim([0 t(end)]);

% figure(3);
% h = animatedline('LineWidth',3);
% grid on; view(3);
% xlabel('X - Downrange'); ylabel('Y - Crossrange'); zlabel('Z - Altitude');
% daspect([1 1 8]); pbaspect([1 1 2]);   
% for k = 1:length(t)
%     addpoints(h, Xe(k), Ye(k), altitude(k));
%     drawnow limitrate
%     pause(0.001)
% end

figure(3); 
plot(t,altitude); grid on; xlabel('time (s)'); ylabel('Altitude');

n = round(linspace(1,length(t),15));
figure(4); 
plot(t,q0,'-o','MarkerIndices',n,'LineWidth',2); hold on; 
plot(t,q1,'--s','MarkerIndices',n,'LineWidth',3);
plot(t,q2,':^','MarkerIndices',n,'LineWidth',2);
plot(t,q3,'-.d','MarkerIndices',n,'LineWidth',2);
plot(t,qnorm);
grid on; xlabel('time (s)'); ylabel('Quaternion'); legend('q_0','q_1','q_2','q_3');

figure(5); 
plot(t,phi,'-o','MarkerIndices',n,'LineWidth',2); hold on; 
plot(t,theta,'--s','MarkerIndices',n,'LineWidth',3);
plot(t,psi,':^','MarkerIndices',n,'LineWidth',2);
grid on; xlabel('time (s)'); ylabel('Euler Angles'); legend('psi','theta','phi');

figure(6); hold on; grid on; view(3);
xlabel('X - Downrange'); ylabel('Y - Crossrange'); zlabel('Z - Altitude');
animateVehicle(t,x);

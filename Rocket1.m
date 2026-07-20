clc; clear; close all;

Xinertial = 0; Yinertial = 0; Zinertial = 0; % Initial inertial position (m)
u0 = 0; v0 = 0; w0 = 0;                      % Initial body translational velocities (m/s)
P0 = 0; Q0 = 0; R0 = 0;                      % Initial body angular velocities (rad/s)
phi0deg   = 0;                               % Initial Euler roll angle (deg)
theta0deg = 60;                              % Initial Euler pitch angle (deg)
psi0deg   = 0;                               % Initial Euler yaw angle (deg)

% Euler angle degrees to radian conversion
phi0   = deg2rad(phi0deg);                   % Initial Euler roll angle (rad)
theta0 = deg2rad(theta0deg);                 % Initial Euler pitch angle (rad)
psi0   = deg2rad(psi0deg);                   % Initial Euler yaw angle (rad)

% Euler angles to quaternion conversion [q0 q1 q2 q3]
q0 = cos(psi0/2)*cos(theta0/2)*cos(phi0/2) - sin(psi0/2)*sin(theta0/2)*sin(phi0/2); % Initial quaternion q0
q1 = sin(theta0/2)*sin(phi0/2)*cos(psi0/2) + sin(psi0/2)*cos(theta0/2)*cos(phi0/2); % Initial quaternion q1
q2 = sin(theta0/2)*cos(psi0/2)*cos(phi0/2) - sin(psi0/2)*sin(phi0/2)*cos(theta0/2); % Initial quaternion q2
q3 = sin(phi0/2)*cos(psi0/2)*cos(theta0/2) + sin(psi0/2)*sin(theta0/2)*cos(phi0/2); % Initial quaternion q3


% eul  = [phi0 theta0 psi0];
% qZYX = eul2quat(eul);
% q    = [q0; q1; q2; q3];

opts = odeset('Events', @stopEvent);                                                % Simulation stop event for when vehicle makes contact with ground
ti = 0; dt = 0.001; tf = 60;                                                        % Simulation time paramters
tsim   = ti:dt:tf;                                                                  % Simulation time vector
x0     = [u0; v0; w0; P0; Q0; R0; Xinertial; Yinertial; Zinertial; q0; q1; q2; q3]; % Initial condition vector 
[t, x] = ode45(@(t,x) rocketEOM(t,x), tsim, x0, opts);                              % Numerical solver 
 u  = x(:,1);   v = x(:,2);   w = x(:,3);             
 P  = x(:,4);   Q = x(:,5);   R = x(:,6);
Xe  = x(:,7);  Ye = x(:,8);  Ze = x(:,9);
q0  = x(:,10); q1 = x(:,11); q2 = x(:,12); q3 = x(:,13); 

altitude = -Ze;

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

figure(4); clf; hold on; grid on; view(3);
xlabel('X - Downrange'); ylabel('Y - Crossrange'); zlabel('Z - Altitude');
animateVehicle(t,x);


function dxdt = rocketEOM(t, x)
    % INPUT: state-vector x  OUTPUT: dxdt
    % State-vector: x = [u v w P Q R X Y Z q0 q1 q2 q3]'
    
    %=====================% State vector indexing %=====================%
    u = x(1); v = x(2); w = x(3);                                                    % Body-frame velocity
    P = x(4); Q = x(5); R = x(6);                                                    % Body angular velocity
    X = x(7); Y = x(8); Z = x(9);                                                    % Inertial-frame position
    q0 = x(10); q1 = x(11); q2 = x(12); q3 = x(13);                                  % Quaternion: scalar-first convention

    Vb     = [u; v; w];                                                              % Translational body velocity vector
    omegab = [P; Q; R];                                                              % Rotational/Angular body velocity vector

    MotorData = readmatrix('AeroTech_HP-H135W.csv');                                 % Measured motor thrust curve
    t_thrust  = MotorData(:,1);                                                      % thrust-curve time samples
    F_thrust  = MotorData(:,2);                                                      % thrust-curve values
    Tm        = interp1(t_thrust, F_thrust, t, 'makima', 0);                         % Interpolate thrust onto simulation time vector

    %=====================% Quaternion Rotation Matrix %=====================%
    % Convert body-frame velocities to inertial-frame velocities
    quat = [q0; q1; q2; q3];                                                         % Quaternion 4-component vector
    quat = quat/norm(quat);                                                          % Normalize quaternion for calculations
    q0 = quat(1); q1 = quat(2); q2 = quat(3); q3 = quat(4);                          % Extract normalized quaternion components

    Cbn = [(q0)^2 + (q1)^2 - (q2)^2 - (q3)^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
           2*(q1*q2 + q0*q3), (q0)^2 - (q1)^2 + (q2)^2 - (q3)^2, 2*(q2*q3 - q0*q1);
           2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0)^2 - (q1)^2 - (q2)^2 + (q3)^2]; % Body-to-inertial rotation matrix (quaternion)
    
    %=====================% Earth-Position rates/Navigation Equations  %=====================%
    % Body Velocity-to-Body Earth/inertial velocity conversion
    % INPUT: u, v, w  OUTPUT: Xdot, Ydot, Zdot
    Ve   = Cbn*Vb; 
    Xdot = Ve(1);
    Ydot = Ve(2);
    Zdot = Ve(3);

    %=====================% Quaternion kinematics %=====================%                                                
    % Required Quaternions to avoid singularities within the Euler Rate
    % equation thus replacing phidot, thetadot, psidot
    % INPUT: q0, q1, q2, q3  OUTPUT: q0dot,q1dot, q2dot, q3dot

    Omega   = [0 -P -Q -R;                                                            
               P  0  R -Q;
               Q -R  0  P;
               R  Q -P  0];                                                          % Omega matrix
    quatdot = (1/2)*Omega*quat;                                                      
    q0dot = quatdot(1); q1dot = quatdot(2); q2dot = quatdot(3); q3dot = quatdot(4);

    %=====================% Body-frame translational equations %=====================%
    % INPUT: u,v,w  OUTPUT: udot,vdot,wdot
    CA = 0.01; CY = 0.01; CN = 0.01; rho = 1.225;                                    % Changing parameters
    m = 0.500; g = 9.81; S = 0.5;                                                    % Constant parameters

    phi   = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));                           % phi Euler angle calculated from quaternion
    theta = asin(max(-1, min(1, 2*(q0*q2 - q3*q1))));                                % theta Euler angle calculated from quaternion
    psi   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));                           % psi Euler angle calculated from quaternion

    if u > 0 
        udot = -(1/m)*CA*0.5*rho*(u)^2*S + R*v - Q*w + (1/m)*Tm - g*sin(theta);      % x-body-axis translational equation
    else 
        udot = (1/m)*CA*0.5*rho*(u)^2*S + R*v - Q*w + (1/m)*Tm - g*sin(theta);
    end
    
    if v > 0 
        vdot = -(1/m)*CY*0.5*rho*(v)^2*S + P*w - R*u + g*sin(phi)*cos(theta);        % y-body-axis translational equation
    else
        vdot = (1/m)*CY*0.5*rho*(v)^2*S + P*w - R*u + g*sin(phi)*cos(theta);
    end

    if w > 0 
        wdot = -(1/m)*CN*0.5*rho*(w)^2*S + Q*u - P*v + g*cos(phi)*cos(theta);        % z-body-axis translational equation
    else
        wdot = (1/m)*CN*0.5*rho*(w)^2*S + Q*u - P*v + g*cos(phi)*cos(theta);
    end

    %=====================% Body-frame rotational equations %=====================%
    % INPUT: P,Q,R  OUTPUT: Pdot,Qdot,Rdot

    ixx = 1; ixy = 0; ixz = 0; 
    iyx = 0; iyy = 1; iyz = 0; 
    izx = 0; izy = 0; izz = 1; 
    I = [ixx -ixy -ixz; 
        -iyx  iyy -iyz; 
        -izx -izy  izz];                                                             % Moment of inertia tensor

    L  = 0;
    % M  = 0.1; 
    % N  = 0.2;

    lref = 1; 
    Cm = 0.001; 
    CM = 0.001; 
    Cn = 0.001; 
    CN = 0.001;

    Vtot = sqrt((u)^2 + (v)^2 + (w)^2);
    q    = (1/2)*rho*(Vtot)^2;
    Mo   = Cm*q*S*lref;                  % moment contribution from AOA in pitch plane and pitch control surface deflection
    % Mq   = CM*Q*q*S*(lref)^2/(2*Vtot); % Pitching moment rate
    Mq   = (CM*Q*(1/2)*rho*(Vtot)*S*(lref)^2)/2;
    M    = Mo + Mq;

    No   = Cn*q*S*lref;
    % Nr   = CN*R*q*S*(lref)^2/(2*Vtot);
    Nr   = (CN*R*(1/2)*rho*(Vtot)*S*(lref)^2)/2;
    N    = No + Nr;

    Pdot = ((Q*R*(iyy - izz))/ixx) + (L/ixx);
    Qdot = ((P*R*(izz - ixx))/iyy) + (M/iyy);
    Rdot = ((P*Q*(ixx - iyy))/izz) + (N/izz);

    %=======================================% 13-state derivative vector %=======================================%
    dxdt = [udot; vdot; wdot; Pdot; Qdot; Rdot; Xdot; Ydot; Zdot; q0dot; q1dot; q2dot; q3dot]; % Supply to ODE45 to be integrated 
end

function animateVehicle(t,x)
    pad = 20;
    xlim([min(x(:,7))-pad max(x(:,7))+pad]);
    ylim([min(x(:,8))-pad max(x(:,8))+pad]);
    % zlim([min(x(:,9))-pad max(x(:,9))+pad]);
    
    % daspect([1 1 8]);
    % pbaspect([1 1 2]);
    
    traj = plot3(nan,nan,nan,'b','LineWidth',2);
    cg   = plot3(x(:,7),x(:,8),x(:,9),'ko','MarkerFaceColor','k','MarkerSize',8);
    
    L = 20;
    c = 'rgm';                   % body axes color
    for i = 1:3
        bodyAxis(i) = quiver3(0,0,0,0,0,0,c(i),'LineWidth',3,'MaxHeadSize',2);
    end
    
    legend('trajectory','CG','x_B forward','y_B right','z_B down');
    
    for k = 1:20:length(t)
        r      = x(k,7:9);        % Xdot, Ydot, Zdot
        window = 90;
        xlim([r(1)-window r(1)+window]);
        ylim([r(2)-window r(2)+window]);
        zlim([-r(3)-window -r(3)+window]);
        % daspect([1 1 1]);
        axis vis3d;
        camproj orthographic;

        q     = x(k,10:13).';    % q0, q1, q2, q3
        q     = q/norm(q);       % normalized quaternion
        Cbn   = quat2Cbn(q);    
        % Tplot = diag([1 1 1]); % converts NED z-down to altitude-up plot
        % B     = L*Tplot*Cbn;
        B     = L*Cbn;
        
        set(traj,'XData',x(1:k,7),'YData',x(1:k,8),'ZData',-x(1:k,9)); % Trajectory drawer
        set(cg,'XData',r(1),'YData',r(2),'ZData',-r(3));               % CG drawer
    
        for i = 1:3
            set(bodyAxis(i),'XData',r(1),'YData',r(2),'ZData',-r(3),'UData',B(1,i),'VData',B(2,i),'WData',-B(3,i));
        end
    
        title(sprintf('t = %.2f s',t(k)));
        drawnow;
        pause(0.001);
    end
end

function Cbn = quat2Cbn(q)
    q  = q/norm(q);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    
    Cbn = [(q0)^2 + (q1)^2 - (q2)^2 - (q3)^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
           2*(q1*q2 + q0*q3), (q0)^2 - (q1)^2 + (q2)^2 - (q3)^2, 2*(q2*q3 - q0*q1);
           2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0)^2 - (q1)^2 - (q2)^2 + (q3)^2];
end

function [value,isterminal,direction] = stopEvent(t, x)
    altitude   = -x(9);      % if x(9) is NED z-down position
    value      = altitude;   % event occurs when this equals zero
    isterminal =  1;         % 1 = stop integration
    direction  = -1;         % detect only increasing crossing
end

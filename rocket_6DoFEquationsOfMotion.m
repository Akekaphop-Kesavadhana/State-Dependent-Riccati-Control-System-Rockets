function dxdt = rocket_6DoFEquationsOfMotion(t, x)
    % INPUT: state-vector (x)  OUTPUT: state-vector time derivative (dxdt)
    % State-vector: x = [u v w P Q R X Y Z q0 q1 q2 q3]'
    
    %=====================% State vector indexing %=====================%
    u = x(1); v = x(2); w = x(3);                                                    % Body-frame velocity
    P = x(4); Q = x(5); R = x(6);                                                    % Body angular velocity
    X = x(7); Y = x(8); Z = x(9);                                                    % Inertial-frame position
    q0 = x(10); q1 = x(11); q2 = x(12); q3 = x(13);                                  % Quaternion: scalar-first convention

    Vb         = [u; v; w];                                                          % Translational body velocity vector
    omegab     = [P; Q; R];                                                          % Rotational/Angular body velocity vector
    quaternion = [q0; q1; q2; q3];                                                   % Quaternion 4-component vector

    MotorData = readmatrix('AeroTech_HP-H135W.csv');                                 % Measured motor thrust curve
    t_thrust  = MotorData(:,1);                                                      % thrust-curve time samples
    F_thrust  = MotorData(:,2);                                                      % thrust-curve values
    Tm        = interp1(t_thrust, F_thrust, t, 'makima', 0);                         % Interpolate thrust onto simulation time vector

    %=====================% Quaternion Rotation Matrix %=====================%
    % Convert body-frame velocities to inertial-frame velocities
    quaternion = quaternion/norm(quaternion);                                        % Normalize quaternion for calculations
    q0 = quaternion(1); q1 = quaternion(2); q2 = quaternion(3); q3 = quaternion(4);  % Extract normalized quaternion components

    Cbn = [(q0)^2 + (q1)^2 - (q2)^2 - (q3)^2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
           2*(q1*q2 + q0*q3), (q0)^2 - (q1)^2 + (q2)^2 - (q3)^2, 2*(q2*q3 - q0*q1);
           2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), (q0)^2 - (q1)^2 - (q2)^2 + (q3)^2]; % Body-to-inertial rotation matrix (quaternion)

    phi   = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));                           % phi Euler angle calculated from quaternion
    theta = asin(max(-1, min(1, 2*(q0*q2 - q3*q1))));                                % theta Euler angle calculated from quaternion
    psi   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));                           % psi Euler angle calculated from quaternion
    % eulerAngleVector = [phi*(180/pi); theta*(180/pi); psi*(180/pi)];
    %=====================% Earth-Position rates/Navigation Equations  %=====================%
    % Body Velocity-to-Body Earth/inertial velocity conversion
    % INPUT: u, v, w  OUTPUT: Xdot, Ydot, Zdot
    Ve   = Cbn*Vb;                                                                   % Body velocity axis rotation to inertial axis
    Xdot = Ve(1);                                                                    % Inertial X translational velocity
    Ydot = Ve(2);                                                                    % Inertial Y translational velocity 
    Zdot = Ve(3);                                                                    % Inertial Z translational velocity
    
    Vedot = [Xdot; Ydot; Zdot];
    %=====================% Quaternion kinematics %=====================%                                                
    % Required Quaternions to avoid singularities within the Euler Rate equation thus replacing phidot, thetadot, psidot
    % INPUT: q0, q1, q2, q3  OUTPUT: q0dot,q1dot, q2dot, q3dot

    Omega   = [0 -P -Q -R;                                                            
               P  0  R -Q;
               Q -R  0  P;
               R  Q -P  0];                                                          % Omega matrix
    quaterniondot = (1/2)*Omega*quaternion;                                                      
    %=====================% Body-frame translational equations %=====================%
    % INPUT: u,v,w  OUTPUT: udot,vdot,wdot
    CA  = 0.1;                                                                       % Aerodynamic Coefficient (Axial) [Non-dimensional]
    CY  = 0.1;                                                                       % Aerodynamic Coefficient (Side) [Non-dimensional]
    CN  = 0.1;                                                                       % Aerodynamic Coefficient (Normal) [Non-dimensional]
    rho = 1.225;                                                                     % Air density [kg/m^3]
    m   = 0.200;                                                                     % Vehicle mass [kg]
    g   = 9.81;                                                                      % Gravitational constant [m/s^2]
    S   = 0.5;                                                                       % Cross sectional area [m^2]

    % if u > 0 
    %     udot = -(1/m)*CA*0.5*rho*(u)^2*S + R*v - Q*w + (1/m)*Tm - g*sin(theta);      % x-body-axis translational equation
    % else 
    %     udot = (1/m)*CA*0.5*rho*(u)^2*S + R*v - Q*w + (1/m)*Tm - g*sin(theta);
    % end
    % 
    % if v > 0 
    %     vdot = -(1/m)*CY*0.5*rho*(v)^2*S + P*w - R*u + g*sin(phi)*cos(theta);        % y-body-axis translational equation
    % else
    %     vdot = (1/m)*CY*0.5*rho*(v)^2*S + P*w - R*u + g*sin(phi)*cos(theta);
    % end
    % 
    % if w > 0 
    %     wdot = -(1/m)*CN*0.5*rho*(w)^2*S + Q*u - P*v + g*cos(phi)*cos(theta);        % z-body-axis translational equation
    % else
    %     wdot = (1/m)*CN*0.5*rho*(w)^2*S + Q*u - P*v + g*cos(phi)*cos(theta);
    % end

    udot = -(1/m)*CA*0.5*rho*abs(u)*u*S + R*v - Q*w + (1/m)*Tm - g*sin(theta); % x-body-axis translational equation
    vdot = -(1/m)*CY*0.5*rho*abs(v)*v*S + P*w - R*u + g*sin(phi)*cos(theta);   % y-body-axis translational equation
    wdot = -(1/m)*CN*0.5*rho*abs(w)*w*S + Q*u - P*v + g*cos(phi)*cos(theta);   % z-body-axis translational equation

    Vbdot = [udot; vdot; wdot];
    %=====================% Body-frame rotational equations %=====================%
    % INPUT: P,Q,R  OUTPUT: Pdot,Qdot,Rdot

    ixx = 1; iyy = 1; izz = 1;                                                        % Moment of inertia (Diagonal only)
                                                             
    % [del1 del3] = SDREControlSystem(Along,Blong,Qlong,Rlong,xlongError);
    % [del2 del4] = SDREControlSystem(Alat,Blat,Qlat,Rlat,xlatError);
    % mix del command into delAileron command
    % mix del command into delRudder command
    % mix del command into delElevator command

    lref = 1;   % Characteristic length [m]
    Cm = 0.001; % Pitching moment coefficient, Cm(Mach, AOA_pitchplane, del_pitch) **LOOKUP TABLE OBTAINED VIA CFD REQ**
    CM = 0.001; % Rate of change of pitching moment, coefficient of moment due to pitch rate **LOOKUP TABLE OBTAINED VIA CFD REQ**
    Cn = 0.001; % Yawing moment coefficient, Cn(Mach, AOA_yawplane, del_yaw) **LOOKUP TABLE OBTAINED VIA CFD REQ**
    CN = 0.001; % Rate of change of yawing moment, coefficient of moment due to yaw rate **LOOKUP TABLE OBTAINED VIA CFD REQ**

    Vtot = sqrt((u)^2 + (v)^2 + (w)^2);                                               % Total airspeed
    q    = (1/2)*rho*(Vtot)^2;                                                        % Dynamic pressure
    
    % L    = q*S*lref*(Cl_alpha_p*alpha_p + Cl_alpha_y*alpha_y + Cl_del_alpha*del_a); % Total rolling moment
   
    Mo   = Cm*q*S*lref;                                                               % Moment contribution from AOA in pitch plane and pitch control surface deflection
    Mq   = (CM*Q*(1/2)*rho*(Vtot)*S*(lref)^2)/2;                                      % Pitching moment rate Mq=CM*Q*q*S*(lref)^2/(2*Vtot);
    M    = Mo + Mq;                                                                   % Total pitching moment contribution
    
    No   = Cn*q*S*lref;                                                               % Moment contribution from AOS in yaw plane and yaw control surface deflection
    Nr   = (CN*R*(1/2)*rho*(Vtot)*S*(lref)^2)/2;                                      % Yawing moment rate Nr=CN*R*q*S*(lref)^2/(2*Vtot); 
    N    = No + Nr;                                                                   % Total yawing moment contribution

    L = 0;
    % M = 0;
    % N = 0;

    Pdot = ((Q*R*(iyy - izz))/ixx) + (L/ixx);
    Qdot = ((P*R*(izz - ixx))/iyy) + (M/iyy);
    Rdot = ((P*Q*(ixx - iyy))/izz) + (N/izz);

    omegabdot = [Pdot; Qdot; Rdot];
    %=====================% 13-state derivative vector %=====================%
    dxdt = [Vbdot; omegabdot; Vedot; quaterniondot];                                  % Supply to ODE45 to be integrated 
end
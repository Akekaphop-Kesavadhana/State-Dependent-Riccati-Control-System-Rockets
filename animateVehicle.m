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
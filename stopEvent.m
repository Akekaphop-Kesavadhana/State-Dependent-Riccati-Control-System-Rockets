function [value,isterminal,direction] = stopEvent(t, x)
    altitude   = -x(9);      % if x(9) is NED z-down position
    value      = altitude;   % event occurs when this equals zero
    isterminal =  1;         % 1 = stop integration
    direction  = -1;         % detect only increasing crossing
end
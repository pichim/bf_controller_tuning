function [Cpi, Cd] = calculate_controllers(PID, Gf_p, Ts)
%CALCULATE_CONTROLLERS  Discrete-time PI and D controller construction
%   [Cpi, Cd] = calculate_controllers(PID, Gf_p, Ts)
%
% PURPOSE
%   - Construct discrete-time PI and D controller components from PID gains
%   - Return controllers as state-space objects for use in closed-loop models
%
% INPUTS
%   - PID   [1 x 3] vector of PID gains [Kp, Ki, Kd]
%   - Gf_p  discrete-time filter for proportional/integral path (e.g. prefilter)
%   - Ts    sample time in seconds
%
% OUTPUTS
%   - Cpi   state-space object representing proportional + integral controller with prefilter
%   - Cd    state-space object representing discrete derivative controller
%
% METHOD
%   1) Extract Kp, Ki, Kd from PID vector
%   2) Form proportional + integral controller:
%        Cpi = Kp*Gf_p + Ki*Ts*tf([1 0], [1 -1], Ts)
%      and convert to state-space
%   3) Form derivative controller:
%        Cd  = (Kd/Ts)*tf([1 -1], [1 0], Ts)
%      and convert to state-space
%
% NOTES
%   - Controllers are returned as state-space (ss) objects for numerical stability
%   - The separation into Cpi and Cd is compatible with two-degree-of-freedom loop structures

    Kp = PID(1);
    Ki = PID(2);
    Kd = PID(3);

    Cpi = ss( Kp*Gf_p + Ki*Ts*tf([1 0], [1 -1], Ts) );
    Cd  = ss( Kd/Ts*tf([1 -1], [1 0], Ts) );

end
  

function CL = calculate_closed_loop(Co, Ci, P, Gf, Gd)
%CALCULATE_CLOSED_LOOP  Compute closed-loop transfer functions for a 2-DOF controller
%   CL = calculate_closed_loop(Co, Ci, P, Gf, Gd)
%
% PURPOSE
%   - Form closed-loop sensitivity and complementary sensitivity functions for nested loops
%   - Capture outer/inner loop interactions, disturbance rejection, and noise transfer
%
% INPUTS
%   - Co   outer-loop controller (e.g. proportional gain Kv or PI part)
%   - Ci   inner-loop controller (e.g. current PI or unity)
%   - P    plant transfer function
%   - Gf   additional filter (e.g. derivative or shaping filter)
%   - Gd   derivative/dynamic element (e.g. Cd·d/dt·Gf_d_part)
%
% OUTPUTS (returned in struct CL)
%   - C    overall controller C = Ci*(Gd+Co)*Gf
%   - L    open-loop transfer L = P*C
%   - S    sensitivity S = 1/(1+L)
%   - T    complementary sensitivity from reference w to y_bar
%   - SP   transfer from disturbance d to output y_bar
%   - SC   transfer from noise n to control u
%   - SCw  transfer from reference w to control u
%   - Li   inner-loop open loop
%   - Si   inner-loop sensitivity
%   - Pi   inner-loop closed-loop plant as seen from outer controller
%   - Ti   inner-loop complementary sensitivity to dy/dt
%   - Lo   outer-loop effective open loop
%
% METHOD
%   1) Form overall controller C = Ci*(Gd+Co)*Gf
%   2) Compute open-loop L = P*C and sensitivity S = 1/(1+L)
%   3) Derive closed-loop transfers T, SP, SC, SCw
%   4) Form inner-loop dynamics Li, Si, Pi, Ti
%   5) Compute effective outer loop Lo
%
% NOTES
%   - Implements a general 2-DOF controller structure used in e.g. Betaflight
%   - T+S~=1 because of the two-degree-of-freedom architecture
%
% T + S ~= 1 (does not hold here)
% Co = Cpi, Ci = 1  , Gd = Cd * d/dt * Gf_d_part -> 2dof PID cntrl betaflight
% Co = Kv , Ci = Cpi, Gd =      d/dt * Gf_d_part -> P-PI cntrl

    C  = Ci*(Gd + Co)*Gf; % C, (Cd + Cpi)*Gf
    L  =             P*C; % L
    S  =       1/(1 + L); % S

    % T   = Co*Ci*P*Gf*S; % T  : w  -> y
    T   =    Co*Ci*P*S; % T  : w  -> y_bar
    % SP  =       P*Gf*S; % SP : d  -> y     (from input disturbance)
    SP  =          P*S; % SP : d  -> y_bar (from input disturbance)
    SC  =          C*S; % SC : n  -> u (from noise)
    SCw =      Co*Ci*S;

    Li = Ci*P*Gf*Gd; % Inner loop
    Si = 1/(1 + Li);
    Pi = Ci*P*Gf*Si; % Inner closed loop, seen from the outer cntrl
    Ti =      Li*Si; % Inner closed loop to outbut dy/dt

    Lo = Co*Ci*P*Gf / (1 + Ci*P*Gf*Gd); % Outer loop

    CL.C   = C;
    CL.L   = L;
    CL.S   = S;
    CL.SCw = SCw;

    CL.T  = T;
    CL.SP = SP;
    CL.SC = SC;

    CL.Li = Li;
    CL.Pi = Pi;
    CL.Ti = Ti;
    CL.Si = Si;

    CL.Lo = Lo;

end

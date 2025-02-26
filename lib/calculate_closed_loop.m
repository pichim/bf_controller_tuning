%
% This file is part of pichim's controller tuning framework.
%
% This sofware is free. You can redistribute this software
% and/or modify this software under the terms of the GNU General
% Public License as published by the Free Software Foundation,
% either version 3 of the License, or (at your option) any later
% version.
%
% This software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%
% See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public
% License along with this software.
%
% If not, see <http:%www.gnu.org/licenses/>.
%
%%
function CL = calculate_closed_loop(Co, Ci, P, Gf, Gd)
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
    
    Li = Ci*P*Gf*Gd; % inner loop
    Si = 1/(1 + Li);
    Pi = Ci*P*Gf*Si; % inner closed loop, seen from the outer cntrl
    Ti =      Li*Si; % inner closed loop to outbut dy/dt
    
    Lo = Co*Ci*P*Gf/(1 + Ci*P*Gf*Gd); % outer loop
    
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


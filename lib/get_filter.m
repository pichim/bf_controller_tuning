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
function [G, B, A] = get_filter(filter_type, f_cut, Ts, prewarp)
% G = get_filter(filter_type, f_cut, Ts, prewarp)
% G = get_filter(filter_type, f_cut, Ts)
%   betaflight filter implementation as is 10.07.2021
% filter_type: string 'pt1', 'pt2', 'pt3', 'biquad', 'notch', 'pt2_custom'
% f_cut      : cutoff frequency in Hz
% Ts         : sampling time
% prewarp    : 0: no prewarping (default)
%              1: prewarping    (only effects pt1,2,3), this is not a
%                                betaflight feature)

    do_pole_matching = false;
    
    % no prewarp in betaflight
    if nargin == 3
        prewarp = 0;
    else
        if do_pole_matching
            prewarp_ptn_fcn = @prewarp_ptn_pole_matching;
        else
            prewarp_ptn_fcn = @prewarp_ptn_mag_matching;
        end
    end
    
    switch filter_type
        case 'pt1'
            % prewarp
            if prewarp
                f_cut = prewarp_ptn_fcn(f_cut, Ts);
            end
            RC = 1/(2*pi*f_cut);
            k  = Ts/(RC + Ts);
            G = tf([k 0], [1 (k-1)], Ts);
        case 'pt2'
            order = 2.0;
            orderCutoffCorrection =  1 / sqrt( 2^(1/order) - 1); % 1.553773974030037
            % prewarp
            if prewarp
                f_cut = prewarp_ptn_fcn(f_cut*orderCutoffCorrection, Ts);
            else
                f_cut = f_cut*orderCutoffCorrection;
            end
            RC = 1/(2*pi*f_cut);
            k  = Ts/(RC + Ts);
            G = tf([k^2 0 0], [1 2*(k-1) (k-1)^2], Ts);
        case 'pt3'
            order = 3.0;
            orderCutoffCorrection =  1 / sqrt( 2^(1/order) - 1); % 1.961459176700620
            % prewarp
            if prewarp
                f_cut = prewarp_ptn_fcn(f_cut*orderCutoffCorrection, Ts);
            else
                f_cut = f_cut*orderCutoffCorrection;
            end
            RC = 1/(2*pi*f_cut);
            k  = Ts/(RC + Ts);
            G = tf([k^3 0 0 0], [1 3*(k-1) 3*(k-1)^2 (k-1)^3], Ts);
        case 'biquad'
            % 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
            % described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
            Q = 1/sqrt(2);
            % prewarp is done implicitly
            omega = 2*pi*f_cut*Ts;
            sn = sin(omega);
            cs = cos(omega);
            alpha = sn / (2 * Q);
            b1 = (1 - cs) / (1 + alpha);
            b0 = b1 * 0.5;
            b2 = b0;
            a1 = -2 * cs / (1 + alpha);
            a2 = (1 - alpha) / (1 + alpha);
            G = tf([b0 b1 b2], [1 a1 a2], Ts);
    %         BIQUAD_Q = 1.0 / sqrt(3.0); % 1.0 / sqrt(2.0);
    %         % setup variables
    %         omega = 2.0 * pi * f_cut * Ts;
    %         alpha = omega / BIQUAD_Q + 1;
    %         b0 = omega^2;
    %         b1 = 0;
    %         b2 = 0;
    %         a1 = -(alpha + 1);
    %         a2 = 1;        
    %         a0 = 1 / (b0 + alpha);
    %         b0 = b0 * a0;
    %         b1 = b1 * a0;
    %         b2 = b2 * a0;
    %         a1 = a1 * a0;
    %         a2 = a2 * a0;
    %         G = tf([b0 b1 b2], [1 a1 a2], Ts);
        case 'notch'
            Q = get_notch_Q(f_cut(2), f_cut(1));
            % prewarp is done implicitly
            omega = 2*pi*f_cut(2)*Ts;
            sn = sin(omega);
            cs = cos(omega);
            alpha = sn / (2 * Q);
            b0 = 1 / (1 + alpha);
            b1 = -2 * cs / (1 + alpha);
            b2 = b0;
            a1 = b1;
            a2 = (1 - alpha) / (1 + alpha);
            G = tf([b0 b1 b2], [1 a1 a2], Ts);
            %damp(G)
        case 'bandpass'
            % Q = getNotchQ(f_cut(2), f_cut(1));
            D = f_cut(1);
            Q = 1/2/D;
            % prewarp is done implicitly
            omega = 2*pi*f_cut(2)*Ts;
            sn = sin(omega);
            cs = cos(omega);
            alpha = sn / (2 * Q);
            b0 = alpha / (1 + alpha);
            b1 = 0;
            b2 = -b0;
            a1 = -2 * cs / (1 + alpha);
            a2 = (1 - alpha) / (1 + alpha);
            G = tf([b0 b1 b2], [1 a1 a2], Ts);
        case 'pt2_custom'
            if prewarp
                f_cut = prewarp_ptn_fcn(f_cut, Ts);
            end
            G = get_filter('pt1', f_cut, Ts) * get_filter('pt1', 1.55*f_cut, Ts);
        case 'pt2_custom2'
            if prewarp
                f_cut = prewarp_ptn_fcn(f_cut, Ts);
            end
            G = get_filter('pt1', f_cut, Ts) * get_filter('pt1', 1.25*f_cut, Ts);
        case 'pt1tustin'
            if prewarp
                f_cut = 1/(pi*Ts)*tan(f_cut*pi*Ts);
            end
            c =  pi*Ts*f_cut;
            b =  c      / (c + 1);
            a = (c - 1) / (c + 1);
            G = tf(b*[1 1], [1 a], Ts);
        case 'phaseComp'
            centerFreqHz   = f_cut(1);
            centerPhaseDeg = f_cut(2);
            omega = 2.0 * pi * centerFreqHz * Ts;
            sn = sin(centerPhaseDeg * pi/180);
            gain = (1 + sn) / (1 - sn);
            alpha = (12 - omega*omega) / (6 * omega * sqrt(gain));  % approximate prewarping (series expansion)
            b0 = 1 + alpha * gain;
            b1 = 2 - b0;
            a1 = 1 - alpha;
            a0 = 1 / (1 + alpha);
            b0 = b0*a0;
            b1 = b1*a0;
            a1 = a1*a0;
            G = tf([b0 b1], [1 a1], Ts);
        case 'leadlag1'
            fz = f_cut(1);
            fp = f_cut(2);
            alpha = fz/fp;
            centerFreqHz = fp * sqrt(alpha);
            centerPhaseDeg = 180/pi*asin( (1 - alpha) / (1 + alpha) );
            G = tf(get_filter('phaseComp', [centerFreqHz, centerPhaseDeg], Ts));
        case 'biquad_gen'
            D = f_cut(2);
            f_cut = f_cut(1);
            if prewarp
                f_cut(1) = 1/(pi*Ts)*tan(f_cut(1)*pi*Ts);
            end
            w0 = 2*pi*f_cut;
            a2 = (Ts^2*w0^2 + 4*D*Ts*w0 + 4);
            a1 = (2*Ts^2*w0^2 - 8) / a2;
            a0 = (Ts^2*w0^2 - 4*D*Ts*w0 + 4) / a2;
            b2 = (Ts^2*w0^2) / a2;
            b1 = (2*Ts^2*w0^2) / a2;
            b0 = (Ts^2*w0^2) / a2;
            G = tf([b2 b1 b0], [1 a1 a0], Ts);
        case 'pt2_gen'
            D = f_cut(2);
            f_cut = f_cut(1);
            if prewarp
                f_cut = prewarp_ptn_fcn(f_cut, Ts);
            end
            w0 = 2*pi*f_cut;
            a2 = (Ts^2*w0^2 + 2*Ts*w0*D + 1);
            a1 = (- 2*Ts*w0*D - 2) / a2;
            a0 = 1 / a2;
            b2 = (Ts^2*w0^2) / a2;
            G = tf([b2 0 0], [1 a1 a0], Ts);  
        otherwise
    end
    
    B = G.num{1};
    A = G.den{1};
    G = ss(G);

end

function f_cut = prewarp_ptn_pole_matching(f_cut, Ts)
    f_cut = 1/(2*pi*Ts)*(exp(2*pi*Ts*f_cut) - 1);
end

function f_cut = prewarp_ptn_mag_matching(f_cut, Ts) 
    k = -(1 - cos(2*pi*f_cut*Ts)) + sqrt((1 - cos(2*pi*f_cut*Ts))*((1 - cos(2*pi*f_cut*Ts)) + 2));
    RC = (Ts - k*Ts) / k; % <- k  = Ts/(RC + Ts);
    f_cut = 1/(2*pi*RC);  % <- RC = 1/(2*pi*f_cut);
end

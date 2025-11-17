function [G, B, A] = get_filter(filter_type, f_cut, Ts)
%GET_FILTER  Discrete-time filter section generator (Betaflight-style)
%   [G, B, A] = get_filter(filter_type, f_cut, Ts)
%
% PURPOSE
%   - Construct a discrete-time IIR filter section by type and cutoff
%   - Return the filter as state-space (G) and as TF coefficients (B, A)
%
% INPUTS
%   - filter_type  char string:
%       'pt1'        first-order low-pass
%       'pt2'        second-order low-pass (Butterworth target, cutoff corrected)
%       'pt3'        third-order low-pass (Butterworth target, cutoff corrected)
%       'biquad'     2nd-order Butterworth biquad (Q = 1/sqrt(2))
%       'notch'      2nd-order notch, Q from get_notch_Q([fnotch fcut])
%       'phaseComp'  first-order phase compensator at center frequency/phase
%       'leadlag1'   lead-lag defined by zero fz and pole fp
%   - f_cut        cutoff/params in Hz
%       'pt1'/'pt2'/'pt3'/'biquad' : scalar fc
%       'notch'                    : [fcut fnotch]
%       'phaseComp'                : [centerFreqHz centerPhaseDeg]
%       'leadlag1'                 : [fz fp]
%   - Ts           sample time [s]
%
% OUTPUTS
%   - G  state-space model of the section (discrete-time, Ts)
%   - B  numerator coefficients of the corresponding tf
%   - A  denominator coefficients of the corresponding tf
%
% METHOD
%   1) For 'pt1' compute k = Ts/(RC+Ts), RC = 1/(2πfc), set G(z) = k z^-1 / (1 + (k-1) z^-1)
%   2) For 'pt2'/'pt3' apply Butterworth order-dependent cutoff correction so fc is at –3 dB
%   3) For 'biquad' use standard digital Butterworth with Q = 1/√2 (TI SLA A447 form)
%   4) For 'notch' compute Q via get_notch_Q and build a normalized biquad notch
%   5) For 'phaseComp' compute gain/alpha from desired center phase, approximate prewarping
%   6) For 'leadlag1' derive equivalent 'phaseComp' parameters from zero/pole pair
%   7) Return both tf coefficients (B,A) and a state-space realization G
%
% NOTES
%   - All sections are discrete-time and created with sample time Ts
%   - 'pt2'/'pt3' use orderCutoffCorrection so specified fc matches analog –3 dB target
%   - 'phaseComp' uses a series-expansion prewarp approximation around the center frequency
%   - get_notch_Q and get_filter recursion ('leadlag1'→'phaseComp') must be available on path

    switch filter_type
        case 'pt1'
            % prewarp
            RC = 1/(2*pi*f_cut);
            k  = Ts/(RC + Ts);
            G = tf([k 0], [1 (k-1)], Ts);
        case 'pt2'
            order = 2.0;
            orderCutoffCorrection =  1 / sqrt( 2^(1/order) - 1); % 1.553773974030037
            f_cut = f_cut*orderCutoffCorrection;
            RC = 1/(2*pi*f_cut);
            k  = Ts/(RC + Ts);
            G = tf([k^2 0 0], [1 2*(k-1) (k-1)^2], Ts);
        case 'pt3'
            order = 3.0;
            orderCutoffCorrection =  1 / sqrt( 2^(1/order) - 1); % 1.961459176700620
            f_cut = f_cut*orderCutoffCorrection;
            RC = 1/(2*pi*f_cut);
            k  = Ts/(RC + Ts);
            G = tf([k^3 0 0 0], [1 3*(k-1) 3*(k-1)^2 (k-1)^3], Ts);
        case 'biquad'
            % 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
            % described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
            Q = 1/sqrt(2);
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
        case 'notch'
            Q = get_notch_Q(f_cut(2), f_cut(1));
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

        otherwise
            warning(['filter_type not valid']);
    end

    B = G.num{1};
    A = G.den{1};
    G = ss(G);

end

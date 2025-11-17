function [exc, fchirp, sinarg] = get_chirp_signals(f0, f1, t1, Ts)
%GET_CHIRP_SIGNALS  Generate logarithmic chirp excitation, instantaneous freq, and phase
%   [exc, fchirp, sinarg] = get_chirp_signals(f0, f1, t1, Ts)
%   [exc, fchirp, sinarg] = get_chirp_signals(f0, f1, t1, time)
%
% PURPOSE
%   - Produce a log-frequency chirp from f0 to f1 over duration t1
%   - Return excitation exc, instantaneous frequency fchirp, and wrapped phase sinarg
%   - Supports two modes:
%       1) generator mode with scalar Ts (internal loop, sample count N = floor(t1/Ts))
%       2) direct mode with explicit time vector 'time' (same length as outputs)
%
% INPUTS
%   - f0    start frequency in Hz (f0 > 0)
%   - f1    end frequency in Hz   (f1 > 0)
%   - t1    total signal length in seconds (t1 > 0)
%   - Ts    either scalar sample time in seconds OR a time vector (direct mode)
%
% OUTPUTS
%   - exc     [N x 1] excitation signal, cosine of wrapped phase with low-freq scaling
%   - fchirp  [N x 1] instantaneous frequency in Hz
%   - sinarg  [N x 1] wrapped phase angle in radians, in [0, 2π)
%
% METHOD
%   1) Use logarithmic sweep f(t) = f0 * beta^(t) with beta = (f1/f0)^(1/t1)
%   2) Phase proxy sinarg = k0 * f(t) − k1 where k0 = 2π / log(beta), k1 = k0*f0
%   3) Wrap sinarg to [0, 2π), set exc = cos(sinarg)
%   4) For very low frequencies (< 1 Hz), scale exc by f(t) to keep integrated angle bounded
%   5) In generator mode, iterate an internal state machine (chirpInit/chirpUpdate) for N samples
%
% NOTES
%   - Direct mode: pass a time vector instead of Ts to compute samples at arbitrary time stamps
%   - Generator mode: Ts must be scalar, outputs have length N = floor(t1/Ts)
%   - The low-frequency scaling (f < 1 Hz) is intended for gyro angle integration use cases
%   - sinarg here is a wrapped phase proxy consistent with the excitation definition

    if (isscalar(Ts))

        % BF implementation
        chirp = chirpInit(f0, f1, t1, Ts);

        exc = zeros(chirp.N, 1);
        fchirp = zeros(chirp.N, 1);
        sinarg = zeros(chirp.N, 1);

        while (true)

            chirp = chirpUpdate(chirp);
            if (chirp.isFinished)
                break
            end

            exc(chirp.count) = chirp.exc;
            fchirp(chirp.count) = chirp.fchirp;
            sinarg(chirp.count) = chirp.sinarg;
        end

    else

        % Direct version
        time = Ts;
        beta = (f1 / f0)^(1.0 / t1);
        k0 = 2.0 * pi / log(beta);
        k1 = k0 * f0;

        fchirp = f0 * beta.^(time);
        sinarg = k0 * fchirp - k1;

        % Wrap sinarg to 0...2*pi
        sinarg = mod(sinarg, 2.0 * pi);

        % Use cosine so that the angle will oscillate around 0 (integral of gyro)
        exc = cos(sinarg);

        % Frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz (integral of gyro)
        ind = fchirp < 1.0;
        exc(ind) = fchirp(ind) .* exc(ind);

    end

end

% Initialize the chirp signal generator
% f0: start frequency in Hz
% f1: end frequency in Hz
% t1: signal length in seconds
% Ts: sampling time in seconds
function chirp = chirpInit(f0, f1, t1, Ts)

    chirp.f0 = f0;
    chirp.Ts = Ts;
    chirp.N = floor(t1 / chirp.Ts);
    chirp.beta = (f1 / f0)^(1.0 / t1);
    chirp.k0 = 2.0 * pi / log(chirp.beta);
    chirp.k1 = chirp.k0 * chirp.f0;
    chirp = chirpReset(chirp);

end

% Reset the chirp signal generator fully
function chirp = chirpReset(chirp)

    chirp.count = 0;
    chirp.isFinished = false;
    chirpResetSignals(chirp);

end

% Reset the chirp signal generator signals
function chirp = chirpResetSignals(chirp)

    chirp.exc = 0.0;
    chirp.fchirp = 0.0;
    chirp.sinarg = 0.0;

end

% Update the chirp signal generator
function chirp = chirpUpdate(chirp)

    if (chirp.isFinished)

        return

    elseif (chirp.count == chirp.N)

        chirp.isFinished = true;
        chirpResetSignals(chirp);
        return

    else

        chirp.fchirp = chirp.f0 * chirp.beta^(chirp.count * chirp.Ts);
        chirp.sinarg = chirp.k0 * chirp.fchirp - chirp.k1;

        % Wrap sinarg to 0...2*pi
        chirp.sinarg = mod(chirp.sinarg, 2.0 * pi);

        % Use cosine so that the angle will oscillate around 0 (integral of gyro)
        chirp.exc = cos(chirp.sinarg);

        % Frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz (integral of gyro)
        if (chirp.fchirp < 1.0)
            chirp.exc = chirp.fchirp * chirp.exc;
        end
        chirp.count = chirp.count + 1;

        return
    end

end

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
function [exc, fchirp, sinarg] = get_chirp_signals(f0, f1, t1, Ts)
% [exc, fchirp, sinarg] = get_chirp_signals(f0, f1, t1, Ts)
% [exc, fchirp, sinarg] = get_chirp_signals(f0, f1, t1, time)

    if (length(Ts) == 1)

        % bf implementation
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

        % direct version
        time = Ts;
        beta = (f1 / f0)^(1.0 / t1);
        k0 = 2.0 * pi / log(beta);
        k1 = k0 * f0;

        fchirp = f0 * beta.^(time);
        sinarg = k0 * fchirp - k1;

        % wrap sinarg to 0...2*pi
        sinarg = mod(sinarg, 2.0 * pi);

        % use cosine so that the angle will oscillate around 0 (integral of gyro)
        exc = cos(sinarg);
        
        % frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz (integral of gyro)
        ind = fchirp < 1.0;
        exc(ind) = fchirp(ind) .* exc(ind);
        
    end

end

% initialize the chirp signal generator
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

% reset the chirp signal generator fully
function chirp = chirpReset(chirp)

    chirp.count = 0;
    chirp.isFinished = false;
    chirpResetSignals(chirp);

end

% reset the chirp signal generator signals
function chirp = chirpResetSignals(chirp)

    chirp.exc = 0.0;
    chirp.fchirp = 0.0;
    chirp.sinarg = 0.0;

end

% update the chirp signal generator
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

        % wrap sinarg to 0...2*pi
        chirp.sinarg = mod(chirp.sinarg, 2.0 * pi);

        % use cosine so that the angle will oscillate around 0 (integral of gyro)
        chirp.exc = cos(chirp.sinarg);
        
        % frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz (integral of gyro)
        if (chirp.fchirp < 1.0)
            chirp.exc = chirp.fchirp * chirp.exc;
        end
        chirp.count = chirp.count + 1;

        return
    end
    
end
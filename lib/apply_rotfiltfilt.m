function xf = apply_rotfiltfilt(G, sinarg, x)
%APPLY_ROTFILTFILT  Quadrature rotation + zero-phase filtering + back-rotation
%   xf = apply_rotfiltfilt(G, sinarg, x)
%
% PURPOSE
%   - Demodulate a narrowband component by complex rotation
%   - Apply zero-phase IIR filtering to the in-phase and quadrature parts
%   - Remodulate back to the original carrier to obtain a cleaned real signal
%
% INPUTS
%   - G       IIR filter struct with fields G.num{1}, G.den{1} for filtfilt
%   - sinarg  [N x 1] phase argument per sample in radians, e.g. 2*pi*f0*t
%   - x       [N x M] real signal matrix, each column a channel
%
% OUTPUTS
%   - xf      [N x M] real filtered signal after rotate–filter–inverse-rotate
%
% METHOD
%   1) Remove column mean from x
%   2) Form complex phasor p = exp(1j*sinarg)
%   3) Rotate yR = y .* p and yQ = y .* conj(p) to isolate sidebands
%   4) Apply zero-phase filtering yR = filtfilt(G.num{1}, G.den{1}, yR) and same for yQ
%   5) Back-rotate and recombine xf = real((yR.*conj(p) + yQ.*p) * 0.5)
%
% NOTES
%   - Uses filtfilt which requires a stable, causal IIR and typically Signal Processing Toolbox
%   - sinarg must be scalar or match the first dimension of x
%   - The 0.5 scaling is omitted here; relative scaling cancels in ratio-based analyses
%   - Ensure G.den{1}(1) == 1 or normalize the filter coefficients before calling

    % Signal size
    [Nx, nx] = size(x);
    xf = zeros(Nx, nx);
    p = exp(1i * sinarg);

    for i = 1:nx
        % Eliminate mean
        y = x(:,i) - mean(x(:,i));
        yR = y .* p;
        yQ = y .* conj(p);

        % Filtering in transformed coordinates
        yR = filtfilt(G.num{1}, G.den{1}, yR);
        yQ = filtfilt(G.num{1}, G.den{1}, yQ);

        % Transform back
        xf(:,i) = real((yR.*conj(p) + yQ.*p) * 0.5);
    end

end

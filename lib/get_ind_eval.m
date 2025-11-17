function ind_eval = get_ind_eval(sinarg, data, threshold)
%GET_IND_EVAL  Select evaluation intervals based on phase window and signal variance
%   ind_eval = get_ind_eval(sinarg, data, threshold)
%
% PURPOSE
%   - Identify index ranges where sinarg > 0 and data shows sufficient activity
%   - Return a logical mask for samples to include in evaluation
%
% INPUTS
%   - sinarg    [N x 1] phase-like signal in radians (or any scalar sequence)
%   - data      [N x 1] measured signal whose variance is tested per window
%   - threshold scalar variance threshold (default 500)
%
% OUTPUTS
%   - ind_eval  [N x 1] logical indices true where windows pass the variance test
%
% METHOD
%   1) Mark candidate region where sinarg > 0
%   2) Find rising and falling edges of that region via diff to form contiguous windows
%   3) For each window, compute var(data(window)) and compare to threshold
%   4) Accept windows whose variance exceeds threshold and set those indices true
%
% NOTES
%   - Edge detection uses a 0/1 mask and thresholds on diff to be robust to float comparisons
%   - If threshold not provided, defaults to 500
%   - Returns false for all samples if no qualifying window is found

    if nargin == 2
        threshold = 500;
    end

    Ndata = size(sinarg, 1);

    ind_eval_candidate = sinarg > 0;

    signal = zeros(Ndata, 1);
    signal(ind_eval_candidate) = 1;
    dsignal = [0; diff(signal)];

    ind_eval_start = find(dsignal >  0.9);
    ind_eval_end   = find(dsignal < -0.9) - 1;
    Neval = size(ind_eval_start, 1);

    ind_eval = false(Ndata, 1);
    for i = 1:Neval
        ind_verify = ind_eval_start(i):ind_eval_end(i);
        if var(data(ind_verify)) > threshold
            ind_eval(ind_verify) = true;
        end
    end

end
